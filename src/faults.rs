


#[derive(Debug)]
pub enum DrvFault {
    Fault,          // Catch all for unimplemented faults. Ideally this will never be returned.
    OverCurrent,    // generic overcurrent fault
    GateDriveFault, // generic gate drive fault
    UnderVoltageLockOut,
    OverTempShutDown,
    HighSideOverCurrentA,
    LowSideOverCurrentA,
    HighSideOverCurrentB,
    LowSideOverCurrentB,
    HighSideOverCurrentC,
    LowSideOverCurrentC,

    PhaseAOverCurrent,
    PhaseBOverCurrent,
    PhaseCOverCurrent,

    OverTempWarning,
    ChargePumpUnderVoltage,

    HighSideGateDriveFaultA,
    LowSideGateDriveFaultA,
    HighSideGateDriveFaultB,
    LowSideGateDriveFaultB,
    HighSideGateDriveFaultC,
    LowSideGateDriveFaultC,
}
