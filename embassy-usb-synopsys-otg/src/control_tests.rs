//! Drive the real control pipe against owned, volatile register backing. The
//! host side is modelled by what the interrupt handler leaves in `State`: an
//! endpoint-0 OUT packet in the endpoint buffer, or a SETUP packet.
use core::future::Future;
use core::task::{Context, Waker};

use embassy_usb_driver::ControlPipe as _;

use super::*;

const GET_LINE_CODING: [u8; 8] = [0xa1, 0x21, 0, 0, 0, 0, 7, 0];
const SET_LINE_CODING: [u8; 8] = [0x21, 0x20, 0, 0, 0, 0, 7, 0];
const LINE_CODING_9600: [u8; 7] = [0x80, 0x25, 0, 0, 0, 0, 8];
const LINE_CODING_115200: [u8; 7] = [0x00, 0xc2, 0x01, 0, 0, 0, 8];

struct Fixture {
    _memory: Vec<u32>,
    regs: Otg,
    state: &'static State<1>,
}

impl Fixture {
    fn new() -> Self {
        let mut memory = vec![0_u32; 0x4000 / 4];
        // SAFETY: every accessed register, including FIFO0, lies in this owned,
        // aligned allocation, which outlives every endpoint built from it.
        let regs = unsafe { Otg::from_ptr(memory.as_mut_ptr().cast()) };
        let state: &'static State<1> = Box::leak(Box::new(State::new()));
        let buffer: &'static mut [u8; 64] = Box::leak(Box::new([0; 64]));
        // SAFETY: the buffer is leaked, so it outlives the state that points to it.
        unsafe { *state.ep_states[0].out_buffer.get() = buffer.as_mut_ptr() };
        regs.diepctl(0).write(|w| w.set_usbaep(true));
        regs.doepctl(0).write(|w| w.set_usbaep(true));
        let mut fifo = regs::Dtxfsts(0);
        fifo.set_ineptfsav(16);
        // SAFETY: fake, owned register bank; emulate the read-only FIFO status.
        unsafe { regs.dtxfsts(0).as_ptr().write_volatile(fifo) };
        Self {
            _memory: memory,
            regs,
            state,
        }
    }

    fn pipe(&self) -> ControlPipe<'static> {
        let info = |direction| EndpointInfo {
            addr: EndpointAddress::from_parts(0, direction),
            ep_type: EndpointType::Control,
            max_packet_size: 64,
            interval_ms: 0,
        };
        ControlPipe {
            max_packet_size: 64,
            regs: self.regs,
            setup_state: &self.state.cp_state,
            ep_in: Endpoint {
                _phantom: PhantomData,
                regs: self.regs,
                state: &self.state.ep_states[0],
                fifo_size_words: 16,
                info: info(Direction::In),
            },
            ep_out: Endpoint {
                _phantom: PhantomData,
                regs: self.regs,
                state: &self.state.ep_states[0],
                fifo_size_words: 16,
                info: info(Direction::Out),
            },
        }
    }

    /// What `on_interrupt` does with an endpoint-0 OUT DATA packet: keep it
    /// when the buffer is free, else log `ep_out buffer overflow` and drop it.
    /// Returns whether the packet was kept.
    fn host_out(&self, bytes: &[u8]) -> bool {
        let ep = &self.state.ep_states[0];
        if ep.out_size.load(Ordering::Acquire) != EP_OUT_BUFFER_EMPTY {
            return false;
        }
        // SAFETY: the buffer is free, so the interrupt side owns it.
        unsafe { core::slice::from_raw_parts_mut(*ep.out_buffer.get(), bytes.len()) }.copy_from_slice(bytes);
        ep.out_size.store(bytes.len() as u16, Ordering::Release);
        true
    }

    /// What `on_interrupt` does with a SETUP packet.
    fn host_setup(&self, request: [u8; 8]) {
        let setup = &self.state.cp_state;
        setup.setup_data[0].store(u32::from_ne_bytes(request[0..4].try_into().unwrap()), Ordering::Relaxed);
        setup.setup_data[1].store(u32::from_ne_bytes(request[4..8].try_into().unwrap()), Ordering::Relaxed);
        setup.setup_ready.store(true, Ordering::Release);
    }
}

fn poll<F: Future>(future: core::pin::Pin<&mut F>) -> Poll<F::Output> {
    future.poll(&mut Context::from_waker(Waker::noop()))
}

/// The host gave up on a SET_LINE_CODING after its data packet reached the
/// device, then sent GET_LINE_CODING. That transfer's status stage must not
/// see the abandoned data, which made `data_in` fail with `BufferOverflow` and
/// left the packet in place to drop the host's status packet as an overflow.
#[test]
fn a_new_setup_discards_out_data_the_host_abandoned_before_an_in_status_stage() {
    let fixture = Fixture::new();
    let mut pipe = fixture.pipe();
    assert!(fixture.host_out(&LINE_CODING_9600), "abandoned data stage");
    fixture.host_setup(GET_LINE_CODING);

    assert_eq!(poll(core::pin::pin!(pipe.setup())), Poll::Ready(GET_LINE_CODING));
    assert_eq!(fixture.regs.doeptsiz(0).read().pktcnt(), 1, "one packet rearmed");
    let mut reply = core::pin::pin!(pipe.data_in(&LINE_CODING_115200, true, true));
    assert_eq!(
        poll(reply.as_mut()),
        Poll::Pending,
        "waits for the host's status packet"
    );
    assert!(
        fixture.host_out(&[]),
        "the status packet is kept, not dropped as an overflow"
    );
    assert_eq!(poll(reply.as_mut()), Poll::Ready(Ok(())));
}

/// The same abandoned packet must not stand in for the data stage of the next
/// control OUT transfer, which then dropped its own data as an overflow.
#[test]
fn a_new_setup_discards_out_data_the_host_abandoned_before_an_out_data_stage() {
    let fixture = Fixture::new();
    let mut pipe = fixture.pipe();
    assert!(fixture.host_out(&LINE_CODING_9600), "abandoned data stage");
    fixture.host_setup(SET_LINE_CODING);

    assert_eq!(poll(core::pin::pin!(pipe.setup())), Poll::Ready(SET_LINE_CODING));
    assert!(fixture.host_out(&LINE_CODING_115200), "this transfer's data is kept");
    let mut data = [0; 7];
    assert_eq!(
        poll(core::pin::pin!(pipe.data_out(&mut data, true, true))),
        Poll::Ready(Ok(7))
    );
    assert_eq!(data, LINE_CODING_115200);
}

/// The host gave up on GET_LINE_CODING before its status stage and sent the
/// next SETUP. Endpoint 0 OUT NAKs until that SETUP is taken, so a status wait
/// that ignored it would wait forever and wedge every later control transfer.
#[test]
fn an_in_status_stage_ends_when_the_host_sends_a_new_setup() {
    let fixture = Fixture::new();
    let mut pipe = fixture.pipe();
    fixture.host_setup(GET_LINE_CODING);
    assert_eq!(poll(core::pin::pin!(pipe.setup())), Poll::Ready(GET_LINE_CODING));
    {
        let mut reply = core::pin::pin!(pipe.data_in(&LINE_CODING_115200, true, true));
        assert_eq!(poll(reply.as_mut()), Poll::Pending);
        fixture.host_setup(SET_LINE_CODING);
        assert_eq!(poll(reply.as_mut()), Poll::Ready(Err(EndpointError::Disabled)));
    }
    assert_eq!(poll(core::pin::pin!(pipe.setup())), Poll::Ready(SET_LINE_CODING));
}

/// Likewise for a control OUT whose data stage the host abandoned.
#[test]
fn an_out_data_stage_ends_when_the_host_sends_a_new_setup() {
    let fixture = Fixture::new();
    let mut pipe = fixture.pipe();
    fixture.host_setup(SET_LINE_CODING);
    assert_eq!(poll(core::pin::pin!(pipe.setup())), Poll::Ready(SET_LINE_CODING));
    let mut data = [0; 7];
    {
        let mut read = core::pin::pin!(pipe.data_out(&mut data, true, true));
        assert_eq!(poll(read.as_mut()), Poll::Pending);
        fixture.host_setup(GET_LINE_CODING);
        assert_eq!(poll(read.as_mut()), Poll::Ready(Err(EndpointError::Disabled)));
    }
    assert_eq!(poll(core::pin::pin!(pipe.setup())), Poll::Ready(GET_LINE_CODING));
}

/// A status packet that arrived before the next SETUP still completes the
/// transfer it belongs to.
#[test]
fn an_in_status_stage_completes_when_its_status_packet_precedes_the_next_setup() {
    let fixture = Fixture::new();
    let mut pipe = fixture.pipe();
    fixture.host_setup(GET_LINE_CODING);
    assert_eq!(poll(core::pin::pin!(pipe.setup())), Poll::Ready(GET_LINE_CODING));
    {
        let mut reply = core::pin::pin!(pipe.data_in(&LINE_CODING_115200, true, true));
        assert_eq!(poll(reply.as_mut()), Poll::Pending);
        assert!(fixture.host_out(&[]));
        fixture.host_setup(SET_LINE_CODING);
        assert_eq!(poll(reply.as_mut()), Poll::Ready(Ok(())));
    }
    assert_eq!(poll(core::pin::pin!(pipe.setup())), Poll::Ready(SET_LINE_CODING));
}
