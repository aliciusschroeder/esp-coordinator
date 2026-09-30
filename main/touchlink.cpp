#include "sdkconfig.h"

#if CONFIG_NCP_TOUCHLINK_ON_PERMIT_JOIN

#include "touchlink.h"
#include "zb_ncp.h"
#include <esp_random.h>
#include <string.h>

// The coordinator headers (ZB_COORDINATOR_ROLE -> ZB_COORDINATOR_ONLY) hide the
// inter-PAN TX API although libzboss_stack.zczr.a implements it. Same layout
// as zb_intrp_data_req_t in zboss_api_aps_interpan.h.
extern "C" {
typedef ZB_PACKED_PRE struct {
    zb_uint8_t  dst_addr_mode;
    zb_uint16_t dst_pan_id;
    zb_addr_u   dst_addr;
    zb_uint16_t profile_id;
    zb_uint16_t cluster_id;
    zb_uint8_t  asdu_handle;
} ZB_PACKED_STRUCT tl_intrp_data_req_t;
void zb_intrp_data_request(zb_uint8_t param);

// Inter-PAN RX: af_interpan.c hands every ZLL commissioning frame (profile
// 0xc05e, cluster 0x1000) to zll_process_device_command() with the APS header
// stripped and zb_intrp_data_ind_t as buffer parameter. The build links with
// --wrap=zll_process_device_command so those frames reach us first.
void __real_zll_process_device_command(zb_uint8_t param);
void __wrap_zll_process_device_command(zb_uint8_t param);
}

namespace touchlink {
namespace {

constexpr uint16_t PROFILE_ZLL        = 0xc05e;
constexpr uint16_t CLUSTER_TOUCHLINK  = 0x1000;
constexpr uint8_t  CMD_SCAN_REQ       = 0x00;
constexpr uint8_t  CMD_SCAN_RSP       = 0x01;
constexpr uint32_t SCAN_PERIOD_MS     = 3000;
constexpr size_t   MAX_REPORTED_ZCL   = 96;

bool     s_active;
uint8_t  s_ticks_left;
uint32_t s_trans_id;
uint32_t s_prev_trans_id;
uint16_t s_scans;
uint16_t s_responses;
uint8_t  s_zcl_tsn;

void emit(const uint8_t* data, size_t len) {
    zb_ncp::indication(static_cast<command_id_t>(TOUCHLINK_IND), data, len);
}

void put_u16(uint8_t* p, uint16_t v) { memcpy(p, &v, 2); }
void put_u32(uint8_t* p, uint32_t v) { memcpy(p, &v, 4); }

void tick(zb_uint8_t);

void finish(uint8_t reason) {
    s_active = false;
    ZB_SCHEDULE_APP_ALARM_CANCEL(tick, ZB_ALARM_ANY_PARAM);
    uint8_t ev[6] = {EV_STOP, reason};
    put_u16(ev + 2, s_scans);
    put_u16(ev + 4, s_responses);
    emit(ev, sizeof(ev));
}

void send_scan_req(zb_uint8_t buf) {
    if (!s_active) {
        zb_buf_free(buf);
        return;
    }
    // ZCL: cluster-specific, client->server, no default response; Scan Request
    // with the zigbee/touchlink information bytes zigbee-herdsman uses.
    auto* p = static_cast<uint8_t*>(zb_buf_initial_alloc(buf, 9));
    p[0] = 0x11;
    p[1] = s_zcl_tsn++;
    p[2] = CMD_SCAN_REQ;
    put_u32(p + 3, s_trans_id);
    p[7] = 0x04;  // zigbee info: coordinator, rx on when idle
    p[8] = 0x12;  // touchlink info: address assignment, link initiator

    auto* req = ZB_BUF_GET_PARAM(buf, tl_intrp_data_req_t);
    memset(req, 0, sizeof(*req));
    req->dst_addr_mode = ZB_INTRP_ADDR_NETWORK;
    req->dst_pan_id = 0xffff;
    req->dst_addr.addr_short = ZB_INTRP_BROADCAST_SHORT_ADDR;
    req->profile_id = PROFILE_ZLL;
    req->cluster_id = CLUSTER_TOUCHLINK;
    zb_intrp_data_request(buf);
}

uint32_t new_trans_id() {
    uint32_t id;
    do {
        id = esp_random();
    } while (id == 0);
    return id;
}

void tick(zb_uint8_t) {
    if (!s_active) {
        return;
    }
    if (s_ticks_left == 0) {
        finish(1);
        return;
    }
    --s_ticks_left;
    s_prev_trans_id = s_trans_id;
    s_trans_id = new_trans_id();
    ++s_scans;
    const bool queued = zb_buf_get_out_delayed(send_scan_req) == RET_OK;

    uint8_t ev[8] = {EV_SCAN_TX};
    put_u32(ev + 1, s_trans_id);
    put_u16(ev + 5, s_scans);
    ev[7] = queued ? 0 : 1;
    emit(ev, sizeof(ev));

    ZB_SCHEDULE_APP_ALARM(tick, 0, ZB_MILLISECONDS_TO_SYS_TIMER_INTERVAL(SCAN_PERIOD_MS));
}

void apply_permit_join(zb_uint8_t duration) {
    if (duration == 0) {
        if (s_active) {
            finish(0);
        }
        return;
    }
    ZB_SCHEDULE_APP_ALARM_CANCEL(tick, ZB_ALARM_ANY_PARAM);
    if (!s_active) {
        s_scans = 0;
        s_responses = 0;
    }
    s_active = true;
    s_ticks_left = duration * 1000u / SCAN_PERIOD_MS;
    if (s_ticks_left == 0) {
        s_ticks_left = 1;
    }

    const uint8_t ev[3] = {EV_START, duration, zb_get_current_channel()};
    emit(ev, sizeof(ev));

    tick(0);
}

// Reports the frame to the host; returns true when it answers our own scan and
// is consumed here instead of reaching the stack's ZLL handler.
bool handle_rx(zb_uint8_t param) {
    const auto* ind = ZB_BUF_GET_PARAM(param, zb_intrp_data_ind_t);
    const auto* zcl = static_cast<const uint8_t*>(zb_buf_begin(param));
    const size_t len = zb_buf_len(param);

    uint8_t ev[15 + MAX_REPORTED_ZCL] = {EV_RX};
    memcpy(ev + 1, ind->src_addr, 8);
    put_u16(ev + 9, ind->src_pan_id);
    ev[11] = static_cast<uint8_t>(ind->rssi);
    ev[12] = ind->link_quality;
    const size_t n = len < MAX_REPORTED_ZCL ? len : MAX_REPORTED_ZCL;
    ev[13] = static_cast<uint8_t>(n);
    memcpy(ev + 14, zcl, n);
    emit(ev, 14 + n);

    if (len < 3) {
        return false;
    }
    const size_t hdr = (zcl[0] & 0x04) ? 5 : 3;  // manufacturer-specific header
    const bool from_server = zcl[0] & 0x08;
    if (!from_server || len < hdr + 4 || zcl[hdr - 1] != CMD_SCAN_RSP) {
        return false;
    }
    uint32_t trans_id;
    memcpy(&trans_id, zcl + hdr, 4);
    if (trans_id != s_trans_id && trans_id != s_prev_trans_id) {
        return false;
    }
    ++s_responses;
    zb_buf_free(param);
    return true;
}

}  // namespace

void on_permit_join(uint8_t duration) {
    zb_schedule_app_callback(apply_permit_join, duration);
}

}  // namespace touchlink

extern "C" void __wrap_zll_process_device_command(zb_uint8_t param) {
    if (touchlink::s_active && touchlink::handle_rx(param)) {
        return;
    }
    __real_zll_process_device_command(param);
}

#endif  // CONFIG_NCP_TOUCHLINK_ON_PERMIT_JOIN
