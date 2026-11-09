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

    /// The core flags a bus event in GINTSTS. It stays pending until
    /// `Bus::poll` handles it, so a control stage that waits on endpoint 0
    /// instead waits forever.
    fn host_bus_event(&self, event: impl FnOnce(&mut regs::Gintsts)) {
        self.regs.gintsts().modify(|w| event(w));
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

/// Polls with a waker that records whether it was woken.
struct Woken(std::sync::Arc<AtomicBool>);

impl std::task::Wake for Woken {
    fn wake(self: std::sync::Arc<Self>) {
        self.0.store(true, Ordering::Release);
    }
}

/// The host reset the bus while a multi-packet IN data stage waited for the
/// previous packet to go out. Endpoint 0 stays active through a reset, so that
/// wait never ended and `Bus::poll` never ran to handle the reset.
#[test]
fn an_in_data_stage_ends_when_the_host_resets_the_bus() {
    let fixture = Fixture::new();
    let mut pipe = fixture.pipe();
    fixture.host_setup(GET_LINE_CODING);
    assert_eq!(poll(core::pin::pin!(pipe.setup())), Poll::Ready(GET_LINE_CODING));
    assert_eq!(
        poll(core::pin::pin!(pipe.data_in(&[0; 64], true, false))),
        Poll::Ready(Ok(()))
    );
    let mut reply = core::pin::pin!(pipe.data_in(&LINE_CODING_115200, false, true));
    assert_eq!(
        poll(reply.as_mut()),
        Poll::Pending,
        "the host has not taken the first packet"
    );
    fixture.host_bus_event(|w| w.set_usbrst(true));
    assert_eq!(poll(reply.as_mut()), Poll::Ready(Err(EndpointError::Disabled)));
    let diepctl = fixture.regs.diepctl(0).read();
    assert!(diepctl.epdis() && diepctl.snak(), "the armed IN packet is cancelled");
}

/// The host suspended the bus instead of sending the status packet of an IN
/// transfer, which is the register state read from the hung DX0.
#[test]
fn an_in_status_stage_ends_when_the_host_suspends_the_bus() {
    let fixture = Fixture::new();
    let mut pipe = fixture.pipe();
    fixture.host_setup(GET_LINE_CODING);
    assert_eq!(poll(core::pin::pin!(pipe.setup())), Poll::Ready(GET_LINE_CODING));
    let mut reply = core::pin::pin!(pipe.data_in(&LINE_CODING_115200, true, true));
    assert_eq!(
        poll(reply.as_mut()),
        Poll::Pending,
        "waits for the host's status packet"
    );
    fixture.host_bus_event(|w| w.set_usbsusp(true));
    assert_eq!(poll(reply.as_mut()), Poll::Ready(Err(EndpointError::Disabled)));
}

/// The host reset the bus instead of sending the data of a control OUT.
#[test]
fn an_out_data_stage_ends_when_the_host_resets_the_bus() {
    let fixture = Fixture::new();
    let mut pipe = fixture.pipe();
    fixture.host_setup(SET_LINE_CODING);
    assert_eq!(poll(core::pin::pin!(pipe.setup())), Poll::Ready(SET_LINE_CODING));
    let mut data = [0; 7];
    let mut read = core::pin::pin!(pipe.data_out(&mut data, true, true));
    assert_eq!(poll(read.as_mut()), Poll::Pending);
    fixture.host_bus_event(|w| w.set_usbrst(true));
    assert_eq!(poll(read.as_mut()), Poll::Ready(Err(EndpointError::Disabled)));
}

/// The zero-length status packet of a control OUT waits behind an IN packet
/// the host never took; enumeration after a reset must end that wait too.
#[test]
fn an_out_status_stage_ends_when_the_host_finishes_a_new_enumeration() {
    let fixture = Fixture::new();
    let mut pipe = fixture.pipe();
    fixture.regs.diepctl(0).modify(|w| w.set_epena(true));
    let mut accept = core::pin::pin!(pipe.accept());
    assert_eq!(poll(accept.as_mut()), Poll::Pending);
    fixture.host_bus_event(|w| w.set_enumdne(true));
    assert_eq!(poll(accept.as_mut()), Poll::Ready(()));
}

/// A bus event that is already pending when a stage begins ends it before it
/// arms endpoint 0, so the stage does not queue a packet for a reset bus.
#[test]
fn a_control_stage_does_not_arm_endpoint_0_while_a_bus_event_is_pending() {
    let fixture = Fixture::new();
    let mut pipe = fixture.pipe();
    fixture.host_bus_event(|w| w.set_usbrst(true));
    assert_eq!(
        poll(core::pin::pin!(pipe.data_in(&LINE_CODING_115200, true, true))),
        Poll::Ready(Err(EndpointError::Disabled))
    );
    assert!(!fixture.regs.diepctl(0).read().epena(), "nothing armed");
    assert_eq!(fixture.regs.dieptsiz(0).read().xfrsiz(), 0, "nothing queued");
}

/// The interrupt masks a bus event until `Bus::poll` handles it. It must also
/// wake the waiting control stage, or that stage never sees the event.
#[test]
fn the_interrupt_wakes_a_control_stage_when_it_masks_a_bus_event() {
    let fixture = Fixture::new();
    let mut pipe = fixture.pipe();
    fixture.host_setup(GET_LINE_CODING);
    assert_eq!(poll(core::pin::pin!(pipe.setup())), Poll::Ready(GET_LINE_CODING));
    let mut reply = core::pin::pin!(pipe.data_in(&LINE_CODING_115200, true, true));
    let woken = std::sync::Arc::new(AtomicBool::new(false));
    let waker = Waker::from(std::sync::Arc::new(Woken(woken.clone())));
    assert_eq!(reply.as_mut().poll(&mut Context::from_waker(&waker)), Poll::Pending);
    fixture.host_bus_event(|w| w.set_usbsusp(true));
    // SAFETY: fake, owned register bank with no RX data or endpoint interrupts.
    unsafe { on_interrupt(fixture.regs, fixture.state, 1) };
    assert!(woken.load(Ordering::Acquire), "the status stage was woken");
    assert_eq!(poll(reply.as_mut()), Poll::Ready(Err(EndpointError::Disabled)));
}
