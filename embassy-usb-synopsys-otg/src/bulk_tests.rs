//! Exercise the real transfer writer against owned, volatile register backing.
use core::future::Future;
use core::task::{Context, Waker};

use super::*;

fn endpoint(state: &State<2>, regs: Otg, fifo_words: u16) -> Endpoint<'_, In> {
    Endpoint {
        _phantom: PhantomData,
        regs,
        state: &state.ep_states[1],
        fifo_size_words: fifo_words,
        info: EndpointInfo {
            addr: EndpointAddress::from_parts(1, Direction::In),
            ep_type: EndpointType::Bulk,
            max_packet_size: 64,
            interval_ms: 0,
        },
    }
}

fn ready(regs: Otg, fifo_words: u16) {
    regs.diepctl(1).write(|w| w.set_usbaep(true));
    let mut status = regs::Dtxfsts(0);
    status.set_ineptfsav(fifo_words);
    // SAFETY: this is a fake, owned register bank, so emulate hardware updating
    // its read-only FIFO status through the backing allocation.
    unsafe { regs.dtxfsts(1).as_ptr().write_volatile(status) };
}

#[test]
fn bulk_transfer_programs_packet_count_and_bytes_without_changing_mps() {
    let mut memory = vec![0_u32; 0x4000 / 4];
    // SAFETY: every accessed register, including FIFO1, lies in this owned,
    // aligned allocation, which remains alive for the endpoint's lifetime.
    let regs = unsafe { Otg::from_ptr(memory.as_mut_ptr().cast()) };
    let state = State::new();
    let mut ep = endpoint(&state, regs, 64);
    let mut cx = Context::from_waker(Waker::noop());
    for len in [0, 1, 63, 64, 65, 128, 192, 255, 256] {
        ready(regs, 64);
        let bytes = vec![0x5a; len];
        let mut write = core::pin::pin!(ep.write_bulk(&bytes));
        assert_eq!(write.as_mut().poll(&mut cx), Poll::Ready(Ok(())));
        assert_eq!(regs.dieptsiz(1).read().xfrsiz(), len as u32);
        assert_eq!(regs.dieptsiz(1).read().pktcnt(), len.div_ceil(64).max(1) as u16);
        assert!(regs.diepctl(1).read().epena());
    }
    assert_eq!(ep.info.max_packet_size, 64);
}

#[test]
fn rejects_over_capacity_and_preserves_ordinary_packet_write_limit() {
    let mut memory = vec![0_u32; 0x4000 / 4];
    let regs = unsafe { Otg::from_ptr(memory.as_mut_ptr().cast()) };
    let state = State::new();
    let mut cx = Context::from_waker(Waker::noop());
    for (fifo, len) in [(16, 65), (64, 257), (128, 257)] {
        let mut ep = endpoint(&state, regs, fifo);
        let bytes = vec![0; len];
        let mut write = core::pin::pin!(ep.write_bulk(&bytes));
        assert_eq!(
            write.as_mut().poll(&mut cx),
            Poll::Ready(Err(EndpointError::BufferOverflow))
        );
        assert_eq!(regs.dieptsiz(1).read().0, 0);
    }
    let mut ep = endpoint(&state, regs, 64);
    let mut write = core::pin::pin!(ep.write(&[0; 65]));
    assert_eq!(
        write.as_mut().poll(&mut cx),
        Poll::Ready(Err(EndpointError::BufferOverflow))
    );
}

#[test]
fn cancellation_and_disconnect_before_acceptance_do_not_program_transfer() {
    let mut memory = vec![0_u32; 0x4000 / 4];
    let regs = unsafe { Otg::from_ptr(memory.as_mut_ptr().cast()) };
    let state = State::new();
    let mut ep = endpoint(&state, regs, 64);
    let mut cx = Context::from_waker(Waker::noop());
    ready(regs, 64);
    regs.diepctl(1).modify(|w| w.set_epena(true));
    {
        let mut write = core::pin::pin!(ep.write_bulk(&[1; 256]));
        assert!(write.as_mut().poll(&mut cx).is_pending());
        assert_eq!(regs.dieptsiz(1).read().0, 0);
    }
    regs.diepctl(1).write(|_| {});
    {
        let mut write = core::pin::pin!(ep.write_bulk(&[2; 256]));
        assert_eq!(write.as_mut().poll(&mut cx), Poll::Ready(Err(EndpointError::Disabled)));
        assert_eq!(regs.dieptsiz(1).read().0, 0);
    }
    ready(regs, 0);
    {
        let mut write = core::pin::pin!(ep.write_bulk(&[2; 256]));
        assert!(write.as_mut().poll(&mut cx).is_pending());
        regs.diepctl(1).write(|_| {});
        assert_eq!(write.as_mut().poll(&mut cx), Poll::Ready(Err(EndpointError::Disabled)));
        assert_eq!(regs.dieptsiz(1).read().0, 0);
    }
    ready(regs, 64);
    let mut write = core::pin::pin!(ep.write_bulk(&[3; 256]));
    assert_eq!(write.as_mut().poll(&mut cx), Poll::Ready(Ok(())));
    assert_eq!(regs.dieptsiz(1).read().xfrsiz(), 256);
}

#[test]
fn composite_cdc_fifo_allocation_fits_and_rejects_oversized_configuration() {
    use embassy_usb_driver::Driver as _;
    let mut memory = vec![0_u32; 0x4000 / 4];
    let regs = unsafe { Otg::from_ptr(memory.as_mut_ptr().cast()) };
    for words in [0, 64, u16::MAX] {
        let state = State::<9>::new();
        let mut rx = [0; 256];
        let mut config = Config::default();
        config.bulk_in_fifo_words = words;
        let mut driver = Driver::new(
            &mut rx,
            OtgInstance {
                regs,
                state: &state,
                fifo_depth_words: 320,
                endpoint_count: 9,
                phy_type: PhyType::InternalFullSpeed,
                extra_rx_fifo_words: 30,
                calculate_trdt_fn: |_| 6,
            },
            config,
        );
        if words == u16::MAX {
            assert!(driver.alloc_endpoint_in(EndpointType::Bulk, None, 64, 0).is_err());
            continue;
        }
        for _ in 0..2 {
            driver.alloc_endpoint_in(EndpointType::Interrupt, None, 8, 255).unwrap();
            driver.alloc_endpoint_out(EndpointType::Bulk, None, 64, 0).unwrap();
            let ep = driver.alloc_endpoint_in(EndpointType::Bulk, None, 64, 0).unwrap();
            assert_eq!(ep.fifo_size_words, words.max(16));
            assert_eq!(ep.info.max_packet_size, 64);
        }
        driver.alloc_endpoint_out(EndpointType::Control, None, 64, 0).unwrap();
        driver.alloc_endpoint_in(EndpointType::Control, None, 64, 0).unwrap();
        assert!(driver.allocated_fifo_words() <= 320);
    }
}
