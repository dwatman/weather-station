#ifndef EEZ_LVGL_UI_SCREENS_H
#define EEZ_LVGL_UI_SCREENS_H

#include <lvgl/lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _objects_t {
    lv_obj_t *main;
    lv_obj_t *panel_in;
    lv_obj_t *panel_out;
    lv_obj_t *panel_press;
    lv_obj_t *indoor_temp;
    lv_obj_t *indoor_humidity;
    lv_obj_t *outdoor_temp;
    lv_obj_t *outdoor_humidity;
    lv_obj_t *indoor_lux;
    lv_obj_t *wifi_dbm;
    lv_obj_t *img_wifi;
    lv_obj_t *clock;
    lv_obj_t *indoor_pressure;
    lv_obj_t *bar1;
    lv_obj_t *bar2;
    lv_obj_t *bar3;
    lv_obj_t *bar4;
    lv_obj_t *bar5;
    lv_obj_t *bar6;
    lv_obj_t *display_bl;
    lv_obj_t *garage_door;
    lv_obj_t *obj0;
} objects_t;

extern objects_t objects;

enum ScreensEnum {
    SCREEN_ID_MAIN = 1,
};

void create_screen_main();
void tick_screen_main();

void tick_screen_by_id(enum ScreensEnum screenId);
void tick_screen(int screen_index);

void create_screens();


#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_SCREENS_H*/