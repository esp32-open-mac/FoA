//! This module implements the background task for the stack.
//!
//! The job of the background runner is to route the received frames to the appropriate interface
//! inputs.
//! Depending on the number of interfaces either the [SingleInterfaceRunner] or [MultiInterfaceRunner] will be used.
//! Internally, these just join the futures of the MAC background task and the interface runners.

use embassy_futures::join::{join, join3};
use log::debug;

use crate::{
    interface::{Interface, InterfaceInput, InterfaceRunner},
    lmac::LMacReceiveEndpoint,
};

/// Run the mac background task.
///
/// This is independent of the number of interfaces being used.
async fn run_internal<'res>(
    receive_endpoint: &mut LMacReceiveEndpoint<'res>,
    if_zero_input: &mut impl InterfaceInput<'res>,
    mut if_one_input: Option<&mut impl InterfaceInput<'res>>,
) -> ! {
    debug!("FoA MAC runner active.");
    loop {
        let received = receive_endpoint.receive().await;
        match (received.interface_zero(), received.interface_one()) {
            (true, false) => {
                if_zero_input.interface_input(received).await;
            }
            (false, true) => {
                if let Some(ref mut if_one_input) = if_one_input {
                    if_one_input.interface_input(received).await;
                }
            }
            (if_zero, if_one) => {
                debug!("Received frame for interface zero: {if_zero}, interface one: {if_one}");
                // I don't know yet, how to handle this.
            }
        };
    }
}

pub struct SingleInterfaceRunner<'res, If: Interface> {
    pub(crate) receive_endpoint: LMacReceiveEndpoint<'res>,
    pub(crate) if_runner: If::RunnerType<'res>,
    pub(crate) if_input: If::InputType<'res>,
}
impl<If: Interface> SingleInterfaceRunner<'_, If> {
    /// Run the background task with two interfaces.
    pub async fn run(&mut self) -> ! {
        join(
            run_internal(
                &mut self.receive_endpoint,
                &mut self.if_input,
                None::<&mut ()>,
            ),
            self.if_runner.run(),
        )
        .await;
        unreachable!()
    }
}
pub struct MultiInterfaceRunner<'res, IfZero: Interface, IfOne: Interface> {
    pub(crate) receive_endpoint: LMacReceiveEndpoint<'res>,
    pub(crate) if_zero_runner: IfZero::RunnerType<'res>,
    pub(crate) if_zero_input: IfZero::InputType<'res>,
    pub(crate) if_one_runner: IfOne::RunnerType<'res>,
    pub(crate) if_one_input: IfOne::InputType<'res>,
}
impl<IfZero: Interface, IfOne: Interface> MultiInterfaceRunner<'_, IfZero, IfOne> {
    /// Run the background task, with two interfaces.
    pub async fn run(&mut self) -> ! {
        join3(
            run_internal(
                &mut self.receive_endpoint,
                &mut self.if_zero_input,
                Some(&mut self.if_one_input),
            ),
            self.if_zero_runner.run(),
            self.if_one_runner.run(),
        )
        .await;
        // The joined future can never complete, since all futures return !.
        unreachable!()
    }
}
