#include "avrcp.h"

#include <stdio.h>


static uint16_t _cid = 0;
static bool _playing = false;

// Full scale until the source says otherwise. A source that does not use AVRCP
// absolute volume attenuates the stream itself, and starting at 0 would put
// that on top of a fixed -42dB here.
static uint8_t _volume = 127;

// ~16 presses to cross the full range
#define VOLUME_STEP 8


static void avrcp_volume_changed(uint8_t volume){
    const btstack_audio_sink_t * audio = btstack_audio_sink_get_instance();
    if (audio){
        audio->set_volume(volume);
    }
}


// btstack emits AVRCP_SUBEVENT_NOTIFICATION_VOLUME_CHANGED from both the target
// and the controller, depending on which side drives the change, and which one
// a given source uses is not something we get to choose. Take it from any of
// our handlers rather than only the target.
static void set_volume_from_source(uint8_t volume){
    if (volume > 127) volume = 127;
    if (volume == _volume) return;

    _volume = volume;
    printf("AVRCP           : Volume %d%% (%d)\n", _volume * 100 / 127, _volume);
    avrcp_volume_changed(_volume);
}


static void connection_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(channel);
    UNUSED(size);

    uint16_t cid;
    uint8_t  status;

    if (packet_type != HCI_EVENT_PACKET) return;
    if (hci_event_packet_get_type(packet) != HCI_EVENT_AVRCP_META) return;

    switch (packet[2]) {
        case AVRCP_SUBEVENT_CONNECTION_ESTABLISHED:
            cid = avrcp_subevent_connection_established_get_avrcp_cid(packet);
            status = avrcp_subevent_connection_established_get_status(packet);
            if (status != ERROR_CODE_SUCCESS){
                // printf("AVRCP: Connection failed, status 0x%02x\n", status);
                _cid = 0;
                return;
            }

            _cid = cid;
            printf("AVRCP           : Connected, cid 0x%02x\n", cid);

            avrcp_target_support_event(cid, AVRCP_NOTIFICATION_EVENT_VOLUME_CHANGED);
            avrcp_target_support_event(cid, AVRCP_NOTIFICATION_EVENT_BATT_STATUS_CHANGED);
            avrcp_target_battery_status_changed(cid, AVRCP_BATTERY_STATUS_EXTERNAL);

            // tell the source where we actually start, otherwise it reads back
            // the 0 btstack defaults to and its slider never matches ours
            avrcp_target_volume_changed(cid, _volume);
        
            // query supported events:
            avrcp_controller_get_supported_events(cid);
            return;
        
        case AVRCP_SUBEVENT_CONNECTION_RELEASED:
            // printf("AVRCP: Channel released: cid 0x%02x\n", avrcp_subevent_connection_released_get_avrcp_cid(packet));
            _cid = 0;
            return;

        case AVRCP_SUBEVENT_NOTIFICATION_VOLUME_CHANGED:
            set_volume_from_source(avrcp_subevent_notification_volume_changed_get_absolute_volume(packet));
            return;

        default:
            break;
    }
}


static void controller_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(channel);
    UNUSED(size);

    uint8_t play_status;

    if (packet_type != HCI_EVENT_PACKET) return;
    if (hci_event_packet_get_type(packet) != HCI_EVENT_AVRCP_META) return;

    if (packet[2] == AVRCP_SUBEVENT_NOTIFICATION_VOLUME_CHANGED){
        set_volume_from_source(avrcp_subevent_notification_volume_changed_get_absolute_volume(packet));
        return;
    }

    if (_cid == 0) return;

    switch (packet[2]) {
        // case AVRCP_SUBEVENT_GET_CAPABILITY_EVENT_ID:
        //     Avrcp::_avrcp->_supported_notifications |= (1 << avrcp_subevent_get_capability_event_id_get_event_id(packet));
        //     break;

        case AVRCP_SUBEVENT_GET_CAPABILITY_EVENT_ID_DONE:
            
            // printf("AVRCP Controller: supported notifications by target:\n");
            // for (event_id = (uint8_t) AVRCP_NOTIFICATION_EVENT_FIRST_INDEX; event_id < (uint8_t) AVRCP_NOTIFICATION_EVENT_LAST_INDEX; event_id++){
            //     printf("   - [%s] %s\n", 
            //         (avrcp_connection->notifications_supported_by_target & (1 << event_id)) != 0 ? "X" : " ", 
            //         avrcp_notification2str((avrcp_notification_event_id_t)event_id));
            // }
            // printf("\n\n");

            // automatically enable notifications
            avrcp_controller_enable_notification(_cid, AVRCP_NOTIFICATION_EVENT_PLAYBACK_STATUS_CHANGED);
            avrcp_controller_enable_notification(_cid, AVRCP_NOTIFICATION_EVENT_NOW_PLAYING_CONTENT_CHANGED);
            avrcp_controller_enable_notification(_cid, AVRCP_NOTIFICATION_EVENT_TRACK_CHANGED);
            break;

        case AVRCP_SUBEVENT_NOTIFICATION_PLAYBACK_STATUS_CHANGED:
            // printf("AVRCP Controller: Playback status changed %s\n", avrcp_play_status2str(avrcp_subevent_notification_playback_status_changed_get_play_status(packet)));
            play_status = avrcp_subevent_notification_playback_status_changed_get_play_status(packet);
            switch (play_status){
                case AVRCP_PLAYBACK_STATUS_PLAYING:
                    _playing = true;
                    break;
                default:
                    _playing = false;
                    break;
            }
            break;

        default:
            break;
    }
}


static void target_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET) return;
    if (hci_event_packet_get_type(packet) != HCI_EVENT_AVRCP_META) return;
    
    switch (packet[2]){
        case AVRCP_SUBEVENT_NOTIFICATION_VOLUME_CHANGED:
            set_volume_from_source(avrcp_subevent_notification_volume_changed_get_absolute_volume(packet));
            break;
        
        // Sources that do not use absolute volume send category 2 button
        // presses instead, and expect us to keep the volume ourselves.
        case AVRCP_SUBEVENT_OPERATION: {
            if (!avrcp_subevent_operation_get_button_pressed(packet)) break;  // ignore the release

            int step;
            switch (avrcp_subevent_operation_get_operation_id(packet)){
                case AVRCP_OPERATION_ID_VOLUME_UP:
                    step = VOLUME_STEP;
                    break;
                case AVRCP_OPERATION_ID_VOLUME_DOWN:
                    step = -VOLUME_STEP;
                    break;
                default:
                    return;
            }

            int volume = _volume + step;
            if (volume < 0) volume = 0;
            if (volume > 127) volume = 127;

            set_volume_from_source((uint8_t) volume);
            avrcp_target_volume_changed(avrcp_subevent_operation_get_avrcp_cid(packet), _volume);
            break;
        }

        default:
            // printf("AVRCP Target    : Event 0x%02x is not parsed\n", packet[2]);
            break;
    }
}


void avrcp_begin() {
    avrcp_init();
    avrcp_controller_init();
    avrcp_target_init();

    avrcp_register_packet_handler(connection_handler);
    avrcp_controller_register_packet_handler(controller_handler);
    avrcp_target_register_packet_handler(target_handler);
}


uint8_t avrcp_get_volume() { 
    return _volume; 
};


bool avrcp_is_connected() { 
    return _cid != 0; 
};


bool avrcp_is_playing() { 
    return _playing; 
};
