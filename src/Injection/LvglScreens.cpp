#include "LvglScreens.h"
#include "LvglTheme.h"
#include "UiFonts.h"
#include "UiBuildConfig.h"
#include "Icons.h"
#include "HomeIcon.h"
#include "NeedleTimeLimits.h"
#include "Ota.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
static constexpr int UI_W=480, UI_H=320, HINT_H=40;
// 조그 화면·배터리 경고창은 320x240 좌표로 짠 것을 480x320으로 늘려 쓴다
static int SX(int v){return v*3/2;}
static int SY(int v){return v*4/3;}
static lv_color_t hex(uint32_t c){return lv_color_hex(c);}
static void loadScreenOnce(lv_obj_t* s){if(s && lv_scr_act()!=s)lv_scr_load(s);}
void lvglLogMemUsage(const char* tag){
#if LV_MEM_CUSTOM == 0
 lv_mem_monitor_t m;lv_mem_monitor(&m);Serial.print("[LVGL] ");Serial.print(tag);Serial.print(" used %=");Serial.println(m.used_pct);
#else
 (void)tag;
#endif
}
static void setLabelText(lv_obj_t* l,const char* t){if(l && t && strcmp(lv_label_get_text(l),t))lv_label_set_text(l,t);}
static void setBgColorIfChanged(lv_obj_t* o,lv_color_t c){if(o && lv_obj_get_style_bg_color(o,0).full!=c.full)lv_obj_set_style_bg_color(o,c,0);}
static void setTextColorIfChanged(lv_obj_t* o,lv_color_t c){if(o && lv_obj_get_style_text_color(o,0).full!=c.full)lv_obj_set_style_text_color(o,c,0);}
static lv_obj_t* panel(lv_obj_t* p,int x,int y,int w,int h,lv_color_t c,int r=0){
 lv_obj_t* o=lv_obj_create(p);lv_obj_remove_style_all(o);lv_obj_clear_flag(o,LV_OBJ_FLAG_SCROLLABLE);
 lv_obj_set_pos(o,x,y);lv_obj_set_size(o,w,h);lv_obj_set_style_bg_color(o,c,0);lv_obj_set_style_bg_opa(o,LV_OPA_COVER,0);lv_obj_set_style_radius(o,r,0);return o;
}
static lv_obj_t* label(lv_obj_t* p,const char* t,int x,int y,const lv_font_t* f,lv_color_t c,int w=0,lv_text_align_t align=LV_TEXT_ALIGN_LEFT){
 lv_obj_t* o=lv_label_create(p);lv_label_set_text(o,t);lv_obj_set_style_text_font(o,f,0);lv_obj_set_style_text_color(o,c,0);lv_obj_set_style_text_align(o,align,0);
 if(w){lv_obj_set_width(o,w);lv_label_set_long_mode(o,LV_LABEL_LONG_CLIP);}lv_obj_set_pos(o,x,y);return o;
}
static lv_obj_t* centered(lv_obj_t* p,const char* t,int y,const lv_font_t* f,lv_color_t c){return label(p,t,24,y,f,c,432,LV_TEXT_ALIGN_CENTER);}
static lv_obj_t* screen(){lv_obj_t* s=lv_obj_create(nullptr);lv_obj_remove_style_all(s);lv_obj_set_style_bg_color(s,LV_CUTE_BG,0);lv_obj_set_style_bg_opa(s,LV_OPA_COVER,0);lv_obj_clear_flag(s,LV_OBJ_FLAG_SCROLLABLE);return s;}
static void icon(lv_obj_t* p,const lv_img_dsc_t* d,int x,int y){lv_obj_t* o=lv_img_create(p);lv_img_set_src(o,d);lv_obj_set_pos(o,x,y);}
static void updateBatteryPill(lv_obj_t* pill,lv_obj_t* l,uint8_t pct){
 lv_color_t c=LV_CUTE_TEXT,bg=LV_CUTE_CARD;
 if(pct<=BAT_CRITICAL_PCT){c=LV_CUTE_RED;bg=LV_CUTE_RED_BG;}else if(pct<=BAT_WARN_PCT){c=LV_CUTE_ORANGE;bg=LV_CUTE_ORANGE_BG;}
 setBgColorIfChanged(pill,bg);setTextColorIfChanged(l,c);for(uint32_t i=1;i<lv_obj_get_child_cnt(pill);i++)setBgColorIfChanged(lv_obj_get_child(pill,i),pct<=BAT_CRITICAL_PCT?LV_CUTE_RED:pct<=BAT_WARN_PCT?LV_CUTE_ORANGE:LV_CUTE_MINT);char b[8];snprintf(b,sizeof(b),"%u%%",pct);setLabelText(l,b);
}
struct Head {lv_obj_t* bat=nullptr;lv_obj_t* batText=nullptr;lv_obj_t* badge=nullptr;lv_obj_t* badgeText=nullptr;};
// 상단 WiFi 아이콘 — 화면마다 하나씩 만들어 두고 연결 상태에 따라 함께 보이거나 숨긴다.
static lv_obj_t* s_wifiIcons[24]; static uint8_t s_wifiIconCount=0; static bool s_wifiShown=false;
static void addWifiIcon(lv_obj_t* s,int x,int dy=0){
 lv_obj_t* o=lv_img_create(s);lv_img_set_src(o,&wifi_icon_small);lv_obj_set_pos(o,x,28+dy);
 lv_obj_set_style_img_recolor(o,LV_CUTE_BLUE,0);lv_obj_set_style_img_recolor_opa(o,LV_OPA_COVER,0);
 if(!s_wifiShown)lv_obj_add_flag(o,LV_OBJ_FLAG_HIDDEN);
 if(s_wifiIconCount<sizeof(s_wifiIcons)/sizeof(s_wifiIcons[0]))s_wifiIcons[s_wifiIconCount++]=o;
}
void lvglSetWifiIndicator(bool on){
 if(on==s_wifiShown)return; s_wifiShown=on;
 for(uint8_t i=0;i<s_wifiIconCount;i++){if(on)lv_obj_clear_flag(s_wifiIcons[i],LV_OBJ_FLAG_HIDDEN);else lv_obj_add_flag(s_wifiIcons[i],LV_OBJ_FLAG_HIDDEN);}
}
// 상단 영역 공통 규칙 (모든 화면 동일 위치 → 화면 전환 시 상단이 흔들리지 않음)
//  - 전체를 3px 아래로: 카드 위 공백과 카드 아래(구분선까지) 공백을 대칭으로
//  - 좌우 끝을 그 화면의 흰 카드 끝선에 맞춤: 배지 헤더(메인/설정)는 16~464, 나머지는 24~456
//  - 제목 왼쪽 아이콘: 배지 헤더는 파란 바탕 캐릭터, 나머지는 투명 아이콘(기본 캐릭터, 주입 화면은 주사기)
static constexpr int HEAD_DY=3;
static Head header(lv_obj_t* s,const char* title,const char* badge=nullptr,lv_color_t accent=LV_CUTE_MINT,lv_color_t soft=LV_CUTE_MINT_BG,const lv_img_dsc_t* titleIcon=&cat_icon){
 Head h;const int L=badge?16:24,R=badge?464:456;
 if(badge){panel(s,L,19+HEAD_DY,40,40,LV_CUTE_BLUE_BG,12);icon(s,&cat_icon,L,19+HEAD_DY);
 h.badge=panel(s,R-208,23+HEAD_DY,108,32,soft,16);h.badgeText=label(h.badge,badge,0,10,&ui_font_17,accent,108,LV_TEXT_ALIGN_CENTER);
 }else icon(s,titleIcon,L+(40-titleIcon->header.w)/2,19+HEAD_DY+(40-titleIcon->header.h)/2);
 label(s,title,L+50,26+HEAD_DY,&ui_font_28,LV_CUTE_TEXT);
 // 배지가 있는 헤더는 제목과 배지 사이, 없는 헤더는 배터리 바로 왼쪽
 addWifiIcon(s,badge?R-240:R-118,HEAD_DY);
 h.bat=panel(s,R-88,23+HEAD_DY,88,32,LV_CUTE_CARD,11);
 h.batText=label(h.bat,"100%",6,9,&ui_font_18,LV_CUTE_TEXT,50);
 panel(h.bat,60,10,18,12,LV_CUTE_MINT,3);panel(h.bat,78,14,3,4,LV_CUTE_MINT,1);
 return h;
}
// 하단 선 아래 안내 문구: 14px, y=294 (선 280 ~ 화면 끝 320 사이 세로 가운데)
static lv_obj_t* footer(lv_obj_t* s,const char* a,const char* b,const char* c){
 panel(s,16,280,448,1,LV_CUTE_SEP);label(s,a,20,294,&ui_font_14,LV_CUTE_SUB,145);label(s,b,166,294,&ui_font_14,LV_CUTE_BLUE,148,LV_TEXT_ALIGN_CENTER);return label(s,c,314,294,&ui_font_14,LV_CUTE_SUB,146,LV_TEXT_ALIGN_RIGHT);
}
static lv_obj_t* hint(lv_obj_t* s,const char* text,lv_color_t c=LV_CUTE_SUB){panel(s,16,280,448,1,LV_CUTE_SEP);return centered(s,text,294,&ui_font_14,c);}
static void stroke(lv_obj_t* p,const lv_point_t* pts,int n,lv_color_t c,int width){lv_obj_t* o=lv_line_create(p);lv_line_set_points(o,pts,n);lv_obj_set_style_line_color(o,c,0);lv_obj_set_style_line_width(o,width,0);lv_obj_set_style_line_rounded(o,true,0);}
// 호밍·업로드 화면 가운데 표시: 둥근 바탕 + 64px 알파 아이콘(색 지정)
static void statusMark(lv_obj_t* s,const lv_img_dsc_t* img,int radius,lv_color_t c,lv_color_t bg){
 panel(s,196,92,88,88,bg,radius);
 lv_obj_t* o=lv_img_create(s);lv_img_set_src(o,img);lv_obj_set_pos(o,208,104);lv_obj_set_style_img_recolor(o,c,0);lv_obj_set_style_img_recolor_opa(o,LV_OPA_COVER,0);
}
static const lv_img_dsc_t* smallIcons[]={&water_icon,&time_icon,&niddle_icon};
static const lv_img_dsc_t* largeIcons[]={&water_icon_large,&time_icon_large,&niddle_icon_large};
static lv_color_t accentFor(int n){return n==0?LV_CUTE_BLUE:n==1?LV_CUTE_GREEN:LV_CUTE_PINK;}
static lv_color_t softFor(int n){return n==0?LV_CUTE_BLUE_BG:n==1?LV_CUTE_GREEN_BG:LV_CUTE_PINK_BG;}
// 니들 선택 표시 여부(시스템 설정 "Needle"). 바뀌면 메인·설정·완료 화면을 다시 만든다.
static bool s_needleOn=NEEDLE_SELECT_DEFAULT_ON;
bool lvglNeedleEnabled(){return s_needleOn;}
struct Ready {lv_obj_t* s=nullptr;Head h;lv_obj_t* val[3]={};} ready;
void lvglShowReady(const InjectionConfig& cfg,SystemState st,uint8_t bat){
 if(!ready.s){ready.s=screen();ready.h=header(ready.s,"Dispenser","READY");panel(ready.s,16,78,448,192,LV_CUTE_CARD,21);
 const char* names[]={"Volume","Time","Needle"};const int order[]={0,2,1};const int rows=s_needleOn?3:2;const int y0=78+(3-rows)*32;
 for(int i=0;i<rows;i++){const int k=s_needleOn?order[i]:i;int y=y0+i*64;panel(ready.s,30,y+13,38,38,softFor(k),12);icon(ready.s,smallIcons[k],33,y+16);label(ready.s,names[k],83,y+22,&ui_font_26,LV_CUTE_TEXT);
 // 글자를 행(구분선 사이) 세로 중앙에 맞춤: 이름 +6px, 숫자 +7px, 단위는 숫자와 같은 기준선(+5px).
 // 숫자는 앞(왼쪽) 정렬 — 341px에서 시작, 단위는 380px.
 ready.val[k]=label(ready.s,"",341,y+19,&ui_font_32,LV_CUTE_TEXT,81,LV_TEXT_ALIGN_LEFT);// 단위(22px): mL은 숫자와 같은 바닥선, G는 그보다 1px 위.
 if(k==0)label(ready.s,"mL",380,y+26,&ui_font_22,LV_CUTE_SUB);else if(k==2)label(ready.s,"G",380,y+25,&ui_font_22,LV_CUTE_SUB);label(ready.s,">",434,y+23,&ui_font_18,LV_CUTE_SUB);if(i<rows-1)panel(ready.s,83,y+63,359,1,LV_CUTE_SEP);}
 footer(ready.s,"PRESS · Align","x2 · Setup","RETURN · Home");lvglLogMemUsage("Ready");}
 char b[24];snprintf(b,sizeof(b),"%u",cfg.volumeMl);setLabelText(ready.val[0],b);snprintf(b,sizeof(b),"%02u:%02u",cfg.timeSec/60,cfg.timeSec%60);setLabelText(ready.val[1],b);if(ready.val[2]){snprintf(b,sizeof(b),"%u",cfg.needleG);setLabelText(ready.val[2],b);}
 setLabelText(ready.h.badgeText,st==STATE_ALIGNED_READY?"ALIGNED":st==STATE_FINISHED?"DONE":"READY");updateBatteryPill(ready.h.bat,ready.h.batText,bat);loadScreenOnce(ready.s);
}
struct Setup {lv_obj_t* s=nullptr;Head h;lv_obj_t* value=nullptr;lv_obj_t* range=nullptr;lv_obj_t* detail=nullptr;};static Setup setupPages[3];
#define SETUP_INFO_COLOR hex(0x8C939D)   // 범위·STEP·MIN 문구: 회색
#define THIN "\xE2\x80\x89"   // 좁은 공백 U+2009 (ui_font_20에 3px로 추가)
static void showSetup(int k,int value,uint8_t bat){
 // 단계: 용량 → (니들) → 시간. 니들을 끄면 2단계
 const int steps=s_needleOn?3:2;const int step=k==0?0:k==2?1:steps-1;Setup& p=setupPages[k];if(!p.s){p.s=screen();char badge[16];snprintf(badge,sizeof(badge),"SETUP %d/%d",step+1,steps);p.h=header(p.s,"Settings",badge,LV_CUTE_BLUE,hex(0xdcecf9));
 {const int w=(448-8*(steps-1))/steps;for(int i=0;i<steps;i++)panel(p.s,16+i*(w+8),82,w,5,i<=step?accentFor(k):LV_CUTE_SEP,3);}
 panel(p.s,16,110,448,110,LV_CUTE_CARD,21);panel(p.s,32,135,60,60,softFor(k),18);icon(p.s,largeIcons[k],41,144);
 label(p.s,k==0?"Volume":k==1?"Time":"Needle",110,155,&ui_font_28,LV_CUTE_TEXT);
 p.value=label(p.s,"",310,145,&ui_font_52,accentFor(k),132,LV_TEXT_ALIGN_LEFT);
 if(k!=1)label(p.s,k==0?"mL":"G",380,155,&ui_font_26,accentFor(k));
 p.range=label(p.s,k==0?"10" THIN "–" THIN "50 mL":k==1?"01:00" THIN "–" THIN "10:00":"21" THIN "–" THIN "25 G",24,240,&ui_font_20,SETUP_INFO_COLOR);
 p.detail=label(p.s,k==0?"Step: 10 mL":k==1?"Step: 10 s":"Step: 1 G",250,240,&ui_font_20,SETUP_INFO_COLOR,206,LV_TEXT_ALIGN_RIGHT);
 footer(p.s,"ROTATE · Edit",step==steps-1?"PRESS · Save":"PRESS · Next","CANCEL · Exit");lvglLogMemUsage("Setup");}
 char b[20];if(k==1)snprintf(b,sizeof(b),"%02d:%02d",value/60,value%60);else snprintf(b,sizeof(b),"%d",value);setLabelText(p.value,b);updateBatteryPill(p.h.bat,p.h.batText,bat);loadScreenOnce(p.s);
}
void lvglShowSetVolume(uint8_t v,uint8_t b){showSetup(0,v,b);}
void lvglShowSetTime(uint16_t t,uint8_t b,uint16_t minimum){
 showSetup(1,t,b);char text[40];
 snprintf(text,sizeof(text),"%02u:%02u" THIN "–" THIN "%02u:%02u",minimum/60,minimum%60,CFG_TIME_MAX_SEC/60,CFG_TIME_MAX_SEC%60);
 setLabelText(setupPages[1].range,text);
 snprintf(text,sizeof(text),"Step: %u s",CFG_TIME_STEP_SEC);setLabelText(setupPages[1].detail,text);
}
void lvglShowSetNeedle(uint8_t n,uint8_t b){
 showSetup(2,n,b);char text[24];uint16_t minimum=needleMinTimeSec(n);
 snprintf(text,sizeof(text),"Min: %02u:%02u",minimum/60,minimum%60);setLabelText(setupPages[2].detail,text);
}
void lvglShowBlank(){static lv_obj_t* s=nullptr;if(!s)s=screen();loadScreenOnce(s);}
struct Status {lv_obj_t* s=nullptr;Head h;lv_obj_t* main=nullptr;lv_obj_t* sub=nullptr;};
static Status homing;
// 글자 두 줄을 표시(원) 아래~카드 아래 사이 세로 가운데에(위아래 여백 18px)
static constexpr int STATUS_MAIN_Y=197,STATUS_SUB_Y=225;
static void makeStatus(Status& p,const char* title,const lv_img_dsc_t* img,int radius,lv_color_t c,lv_color_t bg,const char* main,const char* sub){
 p.s=screen();p.h=header(p.s,title);panel(p.s,24,82,432,174,LV_CUTE_CARD,20);statusMark(p.s,img,radius,c,bg);
 p.main=centered(p.s,main,STATUS_MAIN_Y,&ui_font_22,LV_CUTE_TEXT);p.sub=centered(p.s,sub,STATUS_SUB_Y,&ui_font_17,LV_CUTE_SUB);lvglLogMemUsage(title);
}
void lvglShowHoming(uint8_t bat){if(!homing.s){makeStatus(homing,"Finding home",&home_icon_large,44,hex(0x765eaa),hex(0xeee8f7),"Moving to home position","Preparing your dispenser");footer(homing.s,"","","CANCEL · Stop");}updateBatteryPill(homing.h.bat,homing.h.batText,bat);loadScreenOnce(homing.s);}
// 완료 화면(A안): 왼쪽 큰 체크 원 + 오른쪽 이번 주입 요약(용량·실제 주입 시간·바늘)
// Time은 설정값이 아니라 실제로 걸린 주입 시간(일시정지로 멈춘 시간 제외)이다.
struct Done {lv_obj_t* s=nullptr;Head h;lv_obj_t* val[3]={};};static Done done;
void lvglShowFinished(uint8_t bat,uint8_t volumeMl,float elapsedSec,uint8_t needleG){
 if(!done.s){done.s=screen();done.h=header(done.s,"All done");
  panel(done.s,24,82,432,174,LV_CUTE_CARD,20);
  panel(done.s,52,121,96,96,LV_CUTE_MINT_BG,48);
  {lv_obj_t* p=lv_obj_create(done.s);lv_obj_remove_style_all(p);lv_obj_clear_flag(p,LV_OBJ_FLAG_SCROLLABLE);lv_obj_set_pos(p,65,134);lv_obj_set_size(p,70,70);
   static const lv_point_t ck[]={{13,36},{28,51},{58,20}};stroke(p,ck,3,LV_CUTE_MINT,7);}
  panel(done.s,172,98,1,142,LV_CUTE_SEP);
  const char* names[]={"Volume","Time","Needle"};const int rows=s_needleOn?3:2;const int y0=86+(3-rows)*27;
  for(int k=0;k<rows;k++){const int y=y0+k*56;
   panel(done.s,192,y+9,38,38,softFor(k),12);icon(done.s,smallIcons[k],195,y+12);
   label(done.s,names[k],244,y+16,&ui_font_22,LV_CUTE_SUB);
   done.val[k]=label(done.s,"",300,y+13,&ui_font_26,LV_CUTE_TEXT,136,LV_TEXT_ALIGN_RIGHT);
   if(k<rows-1)panel(done.s,244,y+55,192,1,LV_CUTE_SEP);}
  footer(done.s,"PRESS · Main","","RETURN · Home");lvglLogMemUsage("Done");}
 char b[16];snprintf(b,sizeof(b),"%u mL",(unsigned)volumeMl);setLabelText(done.val[0],b);
 const int es=isfinite(elapsedSec)&&elapsedSec>0?(int)(elapsedSec+0.5f):0;snprintf(b,sizeof(b),"%02d:%02d",es/60,es%60);setLabelText(done.val[1],b);
 if(done.val[2]){snprintf(b,sizeof(b),"%u G",(unsigned)needleG);setLabelText(done.val[2],b);}
 updateBatteryPill(done.h.bat,done.h.batText,bat);loadScreenOnce(done.s);}
// 정지 화면(완료 화면과 같은 구성): 왼쪽 빨간 X 원 + 오른쪽 원인·안내,
// 주입 도중 멈췄으면 아래에 주입량·주입 시간 목록을 붙인다.
struct Stop {lv_obj_t* s=nullptr;Head h;lv_obj_t* main=nullptr;lv_obj_t* sub=nullptr;lv_obj_t* rows=nullptr;lv_obj_t* val[2]={};};static Stop stp;
void lvglShowStopped(uint8_t bat,StopReason reason,float injectedMl,uint8_t volumeMl,float elapsedSec){
 if(!stp.s){stp.s=screen();stp.h=header(stp.s,"Stopped");
  panel(stp.s,24,82,432,174,LV_CUTE_CARD,20);
  panel(stp.s,52,121,96,96,LV_CUTE_RED_BG,48);
  // X: 안티앨리어싱 이미지(x_mark.c)를 원 중앙(100,169)에 — 선으로 그리면 대각선이 계단져 찌그러져 보였다
  {lv_obj_t* o=lv_img_create(stp.s);lv_img_set_src(o,&x_mark);lv_obj_set_pos(o,72,141);
   lv_obj_set_style_img_recolor(o,LV_CUTE_RED,0);lv_obj_set_style_img_recolor_opa(o,LV_OPA_COVER,0);}
  panel(stp.s,172,98,1,142,LV_CUTE_SEP);
  stp.main=label(stp.s,"",192,0,&ui_font_22,LV_CUTE_TEXT,244);
  stp.sub=label(stp.s,"",192,0,&ui_font_17,LV_CUTE_SUB,244);lv_label_set_long_mode(stp.sub,LV_LABEL_LONG_WRAP);
  // 주입 정보 목록(주입 도중 멈춘 경우만 보임)
  stp.rows=lv_obj_create(stp.s);lv_obj_remove_style_all(stp.rows);lv_obj_clear_flag(stp.rows,LV_OBJ_FLAG_SCROLLABLE);lv_obj_set_pos(stp.rows,192,148);lv_obj_set_size(stp.rows,244,104);
  panel(stp.rows,0,0,244,1,LV_CUTE_SEP);
  const char* names[]={"Injected","Time"};const int ic[]={0,1};
  for(int k=0;k<2;k++){const int y=2+k*50;
   panel(stp.rows,0,y+6,38,38,softFor(ic[k]),12);icon(stp.rows,smallIcons[ic[k]],3,y+9);
   label(stp.rows,names[k],52,y+13,&ui_font_22,LV_CUTE_SUB);
   stp.val[k]=label(stp.rows,"",104,y+10,&ui_font_26,LV_CUTE_TEXT,140,LV_TEXT_ALIGN_RIGHT);
   if(k==0)panel(stp.rows,52,y+50,192,1,LV_CUTE_SEP);}
  footer(stp.s,"PRESS · Retry","","RETURN · Home");lvglLogMemUsage("Stopped");}
 const char* main="Motor Stopped";const char* sub="Limit reached or user cancel";char b[48];
 switch(reason){
 case STOP_USER:main="Cancelled by user";sub="Press to re-home and retry";break;
 case STOP_LIMIT:break;
 case STOP_OVERCURRENT:main="Overcurrent";sub="Overcurrent detected";break;
 case STOP_BATTERY:main="Low Battery";snprintf(b,sizeof(b),"Battery %u%% - charge before use",bat);sub=b;break;
 case STOP_HOMING_FAIL:main="Homing Failed";sub="Check home limit switch";break;
 case STOP_MOVE_TIMEOUT:main="Alignment Timeout";sub="Move did not reach target";break;
 case STOP_WATCHDOG:main="Watchdog Reset";sub="Firmware recovered from hang";break;
 case STOP_INCOMPLETE:main="Incomplete";sub="Time ran out before target volume";break;
 case STOP_PAUSE_TIMEOUT:main="Paused Too Long";sub="Injection paused and left idle";break;
 case STOP_BROWNOUT:main="Power Brownout";sub="Supply voltage collapsed - check wiring";break;
 case STOP_PANIC:main="Firmware Crash";sub="Recovered after unexpected reset";break;
 case STOP_SERVO_STALL:main="Plunger Blocked";sub=s_needleOn?"Check needle, line and syringe":"Check line and syringe";break;
 case STOP_SERVO_FAULT:main="Servo Error";sub="Check servo power and cable";break;
 default:break;
 }
 setLabelText(stp.main,main);setLabelText(stp.sub,sub);
 const bool show=isfinite(injectedMl)&&injectedMl>=0&&volumeMl>0;
 if(show){
  char v[24];snprintf(v,sizeof(v),"%.1f mL",injectedMl);setLabelText(stp.val[0],v);
  const int es=isfinite(elapsedSec)&&elapsedSec>0?(int)(elapsedSec+0.5f):0;snprintf(v,sizeof(v),"%02d:%02d",es/60,es%60);setLabelText(stp.val[1],v);
  lv_obj_clear_flag(stp.rows,LV_OBJ_FLAG_HIDDEN);lv_obj_set_y(stp.main,96);lv_obj_set_y(stp.sub,122);
 }else{
  lv_obj_add_flag(stp.rows,LV_OBJ_FLAG_HIDDEN);
  // 원인·안내만: 카드 세로 가운데(안내가 두 줄로 접히면 그만큼 위로)
  lv_obj_update_layout(stp.sub);const int sh=lv_obj_get_height(stp.sub);const int top=82+(174-(26+sh))/2;
  lv_obj_set_y(stp.main,top);lv_obj_set_y(stp.sub,top+28);
 }
 updateBatteryPill(stp.h.bat,stp.h.batText,bat);loadScreenOnce(stp.s);
}
struct Progress {lv_obj_t* s=nullptr;Head h;lv_obj_t* percent=nullptr;lv_obj_t* fill=nullptr;lv_obj_t* a=nullptr;lv_obj_t* b=nullptr;lv_obj_t* hint=nullptr;lv_obj_t* phase=nullptr;lv_obj_t* unit=nullptr;};  // a·b = 왼쪽·오른쪽 값, unit = 오른쪽 항목 이름
static Progress aligning,running;
static void makeProgress(Progress& p,bool run){
 p.s=screen();p.h=header(p.s,run?"Injecting":"Aligning",nullptr,LV_CUTE_MINT,LV_CUTE_MINT_BG,run?&Syringe_icon:&cat_icon);panel(p.s,24,82,432,174,LV_CUTE_CARD,20);p.percent=label(p.s,"0%",56,104,&ui_font_52,LV_CUTE_TEXT,180);p.phase=label(p.s,run?"In progress":"Preparing syringe",242,133,&ui_font_17,LV_CUTE_SUB,193,LV_TEXT_ALIGN_RIGHT);
 panel(p.s,44,166,392,14,hex(0xD2DEEC),7);p.fill=panel(p.s,44,166,1,14,LV_CUTE_BLUE,7);lv_obj_add_flag(p.fill,LV_OBJ_FLAG_HIDDEN);
 // 두 값 칸: 원래처럼 왼쪽 정렬, 열 시작 x=62 / 318 (두 열 묶음이 카드 중앙에 오도록, 간격 넓힘)
 label(p.s,run?"Elapsed":"Volume",62,193,&ui_font_17,LV_CUTE_SUB);p.unit=label(p.s,run?"Injected":"Time",318,193,&ui_font_17,LV_CUTE_SUB,138);
 p.a=label(p.s,"",62,215,&ui_font_26,LV_CUTE_TEXT,196);p.b=label(p.s,"",318,215,&ui_font_26,LV_CUTE_TEXT,130);
 p.hint=hint(p.s,run?"FWD = pause / CANCEL = stop":"Aligning to target position...");lvglLogMemUsage(run?"Running":"Aligning");
}
static void progressValue(Progress& p,float v){if(!isfinite(v))v=0;v=fmaxf(0,fminf(1,v));int w=(int)(392*v);if(w<=0)lv_obj_add_flag(p.fill,LV_OBJ_FLAG_HIDDEN);else {lv_obj_clear_flag(p.fill,LV_OBJ_FLAG_HIDDEN);if(lv_obj_get_width(p.fill)!=w)lv_obj_set_width(p.fill,w);}char b[12];snprintf(b,sizeof(b),"%d%%",(int)(v*100));setLabelText(p.percent,b);}
// 정렬 화면: 기구 좌표(mm) 대신 곧 주입할 용량·시간을 보여 준다(시작 전 확인용)
void lvglShowAligningFrame(uint8_t bat,uint8_t volumeMl,uint16_t timeSec){if(!aligning.s)makeProgress(aligning,false);char b[24];
 snprintf(b,sizeof(b),"%u mL",(unsigned)volumeMl);setLabelText(aligning.a,b);snprintf(b,sizeof(b),"%02u:%02u",(unsigned)(timeSec/60),(unsigned)(timeSec%60));setLabelText(aligning.b,b);progressValue(aligning,0);updateBatteryPill(aligning.h.bat,aligning.h.batText,bat);loadScreenOnce(aligning.s);}
void lvglUpdateAligningValues(float v,uint8_t bat){if(!aligning.s)return;progressValue(aligning,v);updateBatteryPill(aligning.h.bat,aligning.h.batText,bat);}
// 주입 화면 오른쪽 칸: 위치(mm) 대신 주입한 양(mL) = 설정 용량 × 진행률. 전체 용량은 항목 이름에.
static uint8_t s_runVolMl=0;
void lvglShowRunningFrame(uint8_t bat,uint8_t volumeMl){if(!running.s)makeProgress(running,true);
 s_runVolMl=volumeMl;{char u[24];snprintf(u,sizeof(u),"Injected / %u mL",(unsigned)volumeMl);setLabelText(running.unit,u);}progressValue(running,0);setLabelText(running.a,"00:00");lvglSetRunningPaused(false);updateBatteryPill(running.h.bat,running.h.batText,bat);loadScreenOnce(running.s);}
void lvglUpdateRunningValues(float v,float elapsed,uint8_t bat){
 if(!running.s)return;progressValue(running,v);char b[32];int es=(int)elapsed;snprintf(b,sizeof(b),"%02d:%02d",es/60,es%60);setLabelText(running.a,b);
 {const float vv=isfinite(v)?fmaxf(0,fminf(1,v)):0;snprintf(b,sizeof(b),"%.1f mL",s_runVolMl*vv);}setLabelText(running.b,b);
 updateBatteryPill(running.h.bat,running.h.batText,bat);
}
void lvglSetRunningPaused(bool paused){if(!running.s)return;setLabelText(running.phase,paused?"On hold":"In progress");setLabelText(running.hint,paused?"PAUSED - press FWD to resume":"FWD = pause / CANCEL to stop");setTextColorIfChanged(running.hint,paused?LV_CUTE_ORANGE:LV_CUTE_SUB);}
struct SystemMenu {lv_obj_t* s=nullptr;Head h;lv_obj_t* row[SYS_ITEM_COUNT]={};lv_obj_t* val[SYS_ITEM_COUNT]={};};static SystemMenu sys;
void lvglUpdateSysMenu(const InjectionConfig& cfg,uint8_t sel,uint8_t bat){if(!sys.s)return;char b[32];for(int i=0;i<SYS_ITEM_COUNT;i++){setBgColorIfChanged(sys.row[i],i==sel?LV_CUTE_ROW_HILITE:LV_CUTE_CARD);b[0]=0;switch(i){case SYS_BRIGHT:snprintf(b,sizeof(b),"%u%%",cfg.brightPct);break;case SYS_DIM_TIME:if(!cfg.dimSec)snprintf(b,sizeof(b),"Off");else if(cfg.dimSec<60)snprintf(b,sizeof(b),"%u sec",cfg.dimSec);else snprintf(b,sizeof(b),"%u min",cfg.dimSec/60);break;case SYS_WIFI:{OtaState st=otaStatus();if(st==OTA_OFF)snprintf(b,sizeof(b),"Off");else if(otaConnected())snprintf(b,sizeof(b),"On  %s",otaIpString());else if(st==OTA_FAILED)snprintf(b,sizeof(b),otaConfigured()?"On (no network)":"On (not set up)");else snprintf(b,sizeof(b),"On (connecting)");setTextColorIfChanged(sys.val[i],st==OTA_OFF?LV_CUTE_SUB:LV_CUTE_BLUE);break;}case SYS_POWER:snprintf(b,sizeof(b),"%s",lvglBatteryDebugShown()?"On":"Off");setTextColorIfChanged(sys.val[i],lvglBatteryDebugShown()?LV_CUTE_BLUE:LV_CUTE_SUB);break;case SYS_NEEDLE:snprintf(b,sizeof(b),"%s",s_needleOn?"On":"Off");setTextColorIfChanged(sys.val[i],s_needleOn?LV_CUTE_BLUE:LV_CUTE_SUB);break;case SYS_INFO:snprintf(b,sizeof(b),"%s",otaIpString());break;default:break;}setLabelText(sys.val[i],b);}updateBatteryPill(sys.h.bat,sys.h.batText,bat);}
void lvglShowSysMenu(const InjectionConfig& cfg,uint8_t sel,uint8_t bat){if(!sys.s){sys.s=screen();sys.h=header(sys.s,"Settings");panel(sys.s,24,75,432,201,LV_CUTE_CARD,19);const char* names[]={"Wi-Fi","Brightness","Dim After","Power Info","Needle","Info","Exit"};const char* symbols[]={LV_SYMBOL_WIFI,LV_SYMBOL_EYE_OPEN,LV_SYMBOL_IMAGE,LV_SYMBOL_CHARGE,LV_SYMBOL_EDIT,LV_SYMBOL_LIST,LV_SYMBOL_LEFT};
 static_assert(sizeof(names)/sizeof(names[0])==SYS_ITEM_COUNT,"menu label count must match SysMenuItem");
 // 7개 항목이 하단 안내줄(y=280) 위에 들어가도록 줄 간격 28px
 for(int i=0;i<SYS_ITEM_COUNT;i++){int y=76+i*28;sys.row[i]=panel(sys.s,29,y+1,422,26,LV_CUTE_CARD,9);if(i==0){lv_obj_t* o=lv_img_create(sys.s);lv_img_set_src(o,&wifi_icon_small);lv_obj_set_pos(o,40,y+2);lv_obj_set_style_img_recolor(o,LV_CUTE_BLUE,0);lv_obj_set_style_img_recolor_opa(o,LV_OPA_COVER,0);}else label(sys.s,symbols[i],40,y+5,LV_FONT_DEFAULT,LV_CUTE_BLUE);label(sys.s,names[i],73,y+4,&ui_font_18,LV_CUTE_TEXT);sys.val[i]=label(sys.s,"",190,y+5,&ui_font_17,LV_CUTE_SUB,232,LV_TEXT_ALIGN_RIGHT);label(sys.s,">",431,y+6,&ui_font_17,LV_CUTE_SUB);}
 footer(sys.s,"ROTATE · Move","PRESS · Change","CANCEL · Exit");lvglLogMemUsage("System");}lvglUpdateSysMenu(cfg,sel,bat);loadScreenOnce(sys.s);}
static Status ota;static lv_obj_t* otaBar=nullptr;static lv_obj_t* otaExit=nullptr;
void lvglShowOtaFrame(uint8_t bat){if(!ota.s){makeStatus(ota,"Wi-Fi Update",&wifi_icon_large,24,LV_CUTE_BLUE,hex(0xe0edf8),"Connecting...","Waiting for network");panel(ota.s,44,264,392,5,LV_CUTE_SEP,3);otaBar=panel(ota.s,44,264,1,5,LV_CUTE_BLUE,3);lv_obj_add_flag(otaBar,LV_OBJ_FLAG_HIDDEN);otaExit=footer(ota.s,"","","CANCEL · Exit");}updateBatteryPill(ota.h.bat,ota.h.batText,bat);loadScreenOnce(ota.s);lvglUpdateOta();}
void lvglUpdateOta(){if(!ota.s)return;OtaState st=otaStatus();char b[40];if(st==OTA_UPDATING){snprintf(b,sizeof(b),"Updating... %u%%",otaProgressPct());setLabelText(ota.main,b);int w=392*otaProgressPct()/100;lv_obj_clear_flag(otaBar,LV_OBJ_FLAG_HIDDEN);if(lv_obj_get_width(otaBar)!=w)lv_obj_set_width(otaBar,w);}else{setLabelText(ota.main,otaMessage());lv_obj_add_flag(otaBar,LV_OBJ_FLAG_HIDDEN);}setTextColorIfChanged(ota.main,st==OTA_FAILED?LV_CUTE_RED:LV_CUTE_TEXT);
 // 업로드 중: 전원 차단 경고, 취소 안내 숨김(업로드 중에는 입력을 받지 않음)
 const bool up=st==OTA_UPDATING;setLabelText(ota.sub,up?"Do not power off":otaIpString());setTextColorIfChanged(ota.sub,up?LV_CUTE_ORANGE:LV_CUTE_SUB);
 if(up)lv_obj_add_flag(otaExit,LV_OBJ_FLAG_HIDDEN);else lv_obj_clear_flag(otaExit,LV_OBJ_FLAG_HIDDEN);}

static lv_obj_t* jogScreen   = nullptr;
static lv_obj_t* jogBatPill  = nullptr;
static lv_obj_t* jogBatLbl   = nullptr;
static lv_obj_t* jogPosVal   = nullptr;
static lv_obj_t* jogGuardLbl = nullptr;

void lvglShowJogFrame(uint8_t batPct) {
  if (!jogScreen) {
    jogScreen = lv_obj_create(NULL);
    lv_obj_add_style(jogScreen, &style_cute_screen, 0);
    lv_obj_clear_flag(jogScreen, LV_OBJ_FLAG_SCROLLABLE);
    {Head h=header(jogScreen, "JOG FWD"); jogBatPill=h.bat; jogBatLbl=h.batText;}

    lv_obj_t* bigLbl = lv_label_create(jogScreen);
    lv_label_set_text(bigLbl, "Manual Forward");
    lv_obj_set_style_text_font(bigLbl, UI_FONT_2XL, 0);
    lv_obj_set_style_text_color(bigLbl, LV_CUTE_ORANGE, 0);
    lv_obj_align(bigLbl, LV_ALIGN_TOP_MID, SX(0), SY(62));

    lv_obj_t* subLbl = lv_label_create(jogScreen);
    lv_label_set_text(subLbl, "Motor is driving forward");
    lv_obj_set_style_text_font(subLbl, UI_FONT_M, 0);
    lv_obj_set_style_text_color(subLbl, LV_CUTE_SUB, 0);
    lv_obj_align(subLbl, LV_ALIGN_TOP_MID, SX(0), SY(94));

    lv_obj_t* posCard = lv_obj_create(jogScreen);
    lv_obj_remove_style_all(posCard);
    lv_obj_add_style(posCard, &style_cute_card, 0);
    lv_obj_clear_flag(posCard, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(posCard, SX(180), SY(56));
    lv_obj_set_pos(posCard, SX(70), SY(122));
    lv_obj_t* posLbl = lv_label_create(posCard);
    lv_label_set_text(posLbl, "Position");
    lv_obj_set_style_text_color(posLbl, LV_CUTE_SUB, 0);
    lv_obj_set_style_text_font(posLbl, UI_FONT_M, 0);
    lv_obj_set_pos(posLbl, SX(12), SY(6));
    jogPosVal = lv_label_create(posCard);
    lv_obj_set_style_text_font(jogPosVal, UI_FONT_XL, 0);
    lv_obj_set_style_text_color(jogPosVal, LV_CUTE_TEXT, 0);
    lv_obj_set_pos(jogPosVal, SX(12), SY(26));

    jogGuardLbl = lv_label_create(jogScreen);
    lv_obj_set_style_text_font(jogGuardLbl, UI_FONT_S, 0);
    lv_obj_align(jogGuardLbl, LV_ALIGN_TOP_MID, SX(0), SY(184));

    lv_obj_t* hintBar = lv_obj_create(jogScreen);
    lv_obj_remove_style_all(hintBar);
    lv_obj_set_style_bg_color(hintBar, LV_CUTE_HINTBAR_BG, 0);
    lv_obj_set_style_bg_opa(hintBar, LV_OPA_COVER, 0);
    lv_obj_clear_flag(hintBar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(hintBar, UI_W, HINT_H);
    lv_obj_set_pos(hintBar, 0, UI_H - HINT_H);
    lv_obj_t* stopLbl = lv_label_create(hintBar);
    lv_label_set_text(stopLbl, "Press FWD or CANCEL to stop");
    lv_obj_set_style_text_color(stopLbl, LV_CUTE_ORANGE, 0);
    lv_obj_set_style_text_font(stopLbl, UI_FONT_S, 0);
    lv_obj_center(stopLbl);
    lvglLogMemUsage("JOG screen built");
  }
  loadScreenOnce(jogScreen);
  updateBatteryPill(jogBatPill, jogBatLbl, batPct);
}

void lvglUpdateJogValues(float posMm, bool originValid, uint8_t batPct) {
  char buf[16];
  if (originValid) {
    snprintf(buf, sizeof(buf), "%.1f mm", posMm);
    setLabelText(jogPosVal, buf);
    setTextColorIfChanged(jogPosVal, LV_CUTE_TEXT);
    setLabelText(jogGuardLbl, "Soft limit active");
    setTextColorIfChanged(jogGuardLbl, LV_CUTE_SUB);
  } else {
    // 호밍 전에는 위치 기준이 없다
    setLabelText(jogPosVal, "-- mm");
    setTextColorIfChanged(jogPosVal, LV_CUTE_SUB);
    setLabelText(jogGuardLbl, "Not homed - limit switch only");
    setTextColorIfChanged(jogGuardLbl, LV_CUTE_RED);
  }
  updateBatteryPill(jogBatPill, jogBatLbl, batPct);
}

static lv_obj_t* battWarnOverlay = nullptr;

static void ensureBattWarnOverlay() {
  if (battWarnOverlay) return;

  battWarnOverlay = lv_obj_create(lv_layer_top());
  lv_obj_remove_style_all(battWarnOverlay);
  lv_obj_set_style_bg_color(battWarnOverlay, LV_CUTE_RED_BG, 0);
  lv_obj_set_style_bg_opa(battWarnOverlay, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(battWarnOverlay, 2, 0);
  lv_obj_set_style_border_color(battWarnOverlay, LV_CUTE_RED, 0);
  lv_obj_set_style_radius(battWarnOverlay, LV_CUTE_RADIUS_CARD, 0);
  lv_obj_set_size(battWarnOverlay, SX(260), SY(110));
  lv_obj_center(battWarnOverlay);
  lv_obj_clear_flag(battWarnOverlay, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* title = lv_label_create(battWarnOverlay);
  lv_label_set_text(title, "Battery Critical");
  lv_obj_set_style_text_color(title, LV_CUTE_RED, 0);
  lv_obj_set_style_text_font(title, UI_FONT_XL, 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, SX(0), SY(16));

  lv_obj_t* sub = lv_label_create(battWarnOverlay);
  lv_label_set_text(sub, "Charge battery soon");
  lv_obj_set_style_text_color(sub, LV_CUTE_TEXT, 0);
  lv_obj_set_style_text_font(sub, UI_FONT_M, 0);
  lv_obj_align(sub, LV_ALIGN_TOP_MID, SX(0), SY(50));

  lv_obj_add_flag(battWarnOverlay, LV_OBJ_FLAG_HIDDEN);
}

void lvglSetBatteryWarning(bool show) {
  ensureBattWarnOverlay();
  if (show) lv_obj_clear_flag(battWarnOverlay, LV_OBJ_FLAG_HIDDEN);
  else      lv_obj_add_flag(battWarnOverlay, LV_OBJ_FLAG_HIDDEN);
}

static lv_obj_t* battDebugLabel = nullptr;

static void ensureBattDebugLabel() {
  if (battDebugLabel) return;
  battDebugLabel = lv_label_create(lv_layer_top());
  lv_obj_set_style_bg_color(battDebugLabel, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(battDebugLabel, LV_OPA_70, 0);
  lv_obj_set_style_text_color(battDebugLabel, lv_color_white(), 0);
  lv_obj_set_style_text_font(battDebugLabel, UI_FONT_S, 0);
  lv_obj_set_style_pad_all(battDebugLabel, 2, 0);
  // 헤더 왼쪽의 캐릭터 아이콘·제목을 가리지 않게 화면 맨 위 가운데(헤더 위 여백)에 둔다
  lv_obj_align(battDebugLabel, LV_ALIGN_TOP_MID, 0, 0);
  lv_label_set_text(battDebugLabel, "");
  lv_obj_add_flag(battDebugLabel, LV_OBJ_FLAG_HIDDEN);
}

static bool s_powerInfoShown = false;
bool lvglBatteryDebugShown() { return s_powerInfoShown; }
void lvglSetBatteryDebug(bool show) {
  s_powerInfoShown = show;
  ensureBattDebugLabel();
  if (show) lv_obj_clear_flag(battDebugLabel, LV_OBJ_FLAG_HIDDEN);
  else      lv_obj_add_flag(battDebugLabel, LV_OBJ_FLAG_HIDDEN);
}

void lvglUpdateBatteryDebug(float vBus, float iBat, uint8_t pct, int calErr) {
  ensureBattDebugLabel();
  char buf[96];
  snprintf(buf, sizeof(buf),
           "V=%.2f  BAT=%.0fmA  %d%% cal=%d",
           vBus, iBat, pct, calErr);
  setLabelText(battDebugLabel, buf);
  lv_obj_set_style_text_color(battDebugLabel, lv_color_white(), 0);
}

// 센서 무응답은 0V 표시와 구분해 빨간 경고로
void lvglUpdateBatteryDebugNoSensor() {
  ensureBattDebugLabel();
  setLabelText(battDebugLabel, "INA228 NOT FOUND - check I2C wiring/address");
  lv_obj_set_style_text_color(battDebugLabel, LV_CUTE_RED, 0);
}

// 니들 선택 표시 On/Off — 니들 행이 있는 화면은 지워 두고 다음 표시 때 다시 만든다
// 화면을 지우기 전에 그 화면의 WiFi 아이콘을 목록에서 뺀다(지운 객체를 가리키지 않게)
static void dropScreen(lv_obj_t* s){
 if(!s)return;if(lv_scr_act()==s)lvglShowBlank();   // 보이는 화면은 지울 수 없으므로 빈 화면으로 바꾼 뒤 지운다
 uint8_t n=0;
 for(uint8_t i=0;i<s_wifiIconCount;i++)if(lv_obj_get_screen(s_wifiIcons[i])!=s)s_wifiIcons[n++]=s_wifiIcons[i];
 s_wifiIconCount=n;lv_obj_del(s);
}
void lvglSetNeedleEnabled(bool on){
 if(on==s_needleOn)return;s_needleOn=on;
 dropScreen(ready.s);ready=Ready();
 for(auto& p:setupPages){dropScreen(p.s);p=Setup();}
 dropScreen(done.s);done=Done();
}
