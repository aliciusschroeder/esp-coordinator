#pragma once
#include <stdint.h>

// Touchlink initiator (ZLL commissioning cluster over inter-PAN) that runs while
// the host keeps permit-join open. Needed for devices like the IKEA KAJPLATS
// LED2401G5 that only leave their pairing mode for a Touchlink initiator.
//
// Everything stays on the current channel. Progress is reported to the host
// as TOUCHLINK_IND vendor indications, which zigbee-herdsman logs as raw
// frames (`<-- FRAME: ...`) and otherwise ignores.
namespace touchlink {

// Vendor indication id, outside every command range the NCP protocol uses.
constexpr uint16_t TOUCHLINK_IND = 0x0F01;

// TOUCHLINK_IND payload: [event(1) | event data]
enum event_t : uint8_t {
    EV_START     = 0x01, // duration(1) channel(1)
    EV_SCAN_TX   = 0x02, // trans_id(4) scan_no(2) status(1): 0 queued, 1 no buffer
    EV_RX        = 0x03, // src_ieee(8) src_pan(2) rssi(1) lqi(1) zcl_len(1) zcl(zcl_len)
    EV_STOP      = 0x04, // reason(1): 0 permit-join closed, 1 window over; scans(2) responses(2)
};

// Called on every coordinator permit-join request. Any duration > 0 (re)opens
// a scan window of that many seconds, 0 closes it. Safe to call from the app
// task under the ZBOSS lock and from the ZBOSS task.
void on_permit_join(uint8_t duration);

}  // namespace touchlink
