#pragma once
#include <Arduino.h>
#include "Theme.h"

// ══════════════════════════════════════════════════════════
// Motor — STS3250 버스 서보 구동부
//
//   · 위치(getPositionMm/getStepCount)는 서보 엔코더 실측값이다.
//   · 버스 통신과 위치 루프는 별도 태스크(SERVO_TASK_CORE)가 10ms 주기로
//     수행하고, 이 클래스의 메서드는 명령을 걸어 두거나 상태를 읽기만 한다.
//   · 막힘(isStalled)과 서보 고장(isDriverFault)을 따로 알려 준다.
// ══════════════════════════════════════════════════════════
enum HomingState : uint8_t { HOMING_IDLE,HOMING_RUNNING,HOMING_BACKOFF,
                           HOMING_DONE,HOMING_TIMEOUT,HOMING_ERROR };
enum MoveState : uint8_t { MOVE_IDLE,MOVE_RUNNING,MOVE_DONE,MOVE_TIMEOUT,MOVE_ERROR };

// 진단 출력용 서보 상태 스냅샷
struct ServoTelemetry {
  bool     online;        // 최근 피드백 읽기 성공
  bool     echo;          // 어댑터 에코 판별 결과
  uint16_t model;         // 부팅 시 읽은 모델 번호 (0=미확인)
  uint8_t  mode;          // 서보 동작 모드 레지스터
  float    voltageV;
  uint8_t  tempC;
  int16_t  loadPermil;    // ±1000
  uint8_t  status;        // 서보 오류 비트
  float    posMm, refMm, cmdMmS;   // 실측 위치, 궤적 기준 위치, 속도 명령
  uint32_t commErrors;    // 누적 통신 실패 수 (재시도까지 실패한 틱)
  uint32_t commRetries;   // 첫 읽기 실패 후 재시도한 횟수 (재시도로 복구된 것 포함)
};

class Motor {
public:
  void begin();
  void service();
  void startHoming();
  HomingState updateHoming();
  void startMoveTo(float targetMm);
  MoveState updateMoveTo();
  // seekFront: 목표 도착 후 전방 리밋이 안 눌렸으면 FRONT_SEEK_MAX_MM까지 더 밀어 리밋을 찾는다
  //            (목표가 "주사기가 비는 위치"일 때 — Injection.ino의 주입 시작)
  void startDispense(float distanceMm,float durationSec, bool seekFront = false);
  bool updateDispense();
  void pauseDispense();
  void resumeDispense();
  bool isDispensePaused() const { return _paused; }
  unsigned long getPauseElapsedMs() const { return _paused ? millis()-_pauseMs : 0; }
  void stop();
  void emergencyStop();
  void jogForward();
  bool isFrontLimit() const;
  bool isHomeLimit() const;
  bool isSoftLimit() const;
  bool isOriginValid() const;
  bool hasMotionFault() const;
  uint8_t getFaultCode() const;
  bool isDriverFault() const;
  bool isStalled() const;
  void updateCurrentMa(float mA);
  bool isOverCurrent() const { return _overCurrent; }
  bool isCurrentSaturated() const { return _currentMa>=INA228_SATURATED_MA; }
  bool isDispenseIncomplete() const { return _incomplete; }
  bool isDispenseComplete() const { return _complete; }       // 마지막 주입이 정상 완료(전방 리밋 끝 포함)
  bool isDispenseEndedAtFront() const { return _endedAtFront; }
  // 목표 도착 후 리밋을 찾아 더 민 거리(mm)와, 리밋을 못 찾았는지 — 완료 로그·보정용
  float dispenseSeekMm() const { return _seekMm; }
  bool  dispenseSeekMissed() const { return _seekMissed; }
  // 마지막 주입이 전방 리밋에서 끝났다면 리밋이 눌린 위치(mm), 아니면 NAN — 끝 위치 학습용
  float dispenseFrontPressMm() const { return _frontPressMm; }
  // 지금 눌린 전방 리밋이 "주입 끝(완료 허용 범위 안)"이라 완료로 처리될 상황인가
  bool frontLimitIsDispenseEnd() const;
  void clearFaults();
  // INA228 ALERT 인터럽트에서 호출 (플래그만 세움, ISR 안전)
  static void IRAM_ATTR currentAlertISR();
  bool isCurrentAlert() const;      // ALERT로 차단된 상태(clearFaults 전까지 유지)
  static bool isBatteryBlock(uint8_t pct) { return pct<=BAT_BLOCK_PCT; }
  float getPositionMm() const;
  float getProgress() const;
  float getElapsedSec() const;
  float getMoveProgress() const;
  long getStepCount() const; // 서보 엔코더 실측 위치(step, 원점 기준)
  ServoTelemetry getTelemetry() const;
private:
  bool configureServo();
  bool launchProfile(long target,double seconds,uint8_t guard); // 원점 기준 목표 step
  bool launchVelocity(float speedMmS,float maxDistMm,uint8_t guard,bool endOnFront=false);
  void failMotion(bool electrical=false);
  HomingState _homing=HOMING_IDLE;
  MoveState _move=MOVE_IDLE;
  unsigned long _homeMs=0,_releaseMs=0,_moveMs=0,_pauseMs=0;
  long _moveStart=0,_moveTarget=0,_doseStart=0,_doseTarget=0;
  double _frontEndTol=0;   // 끝 무렵 전방 리밋 = 완료로 보는 범위(step), startDispense에서 계산
  uint64_t _doseTotalUs=0,_dosePriorUs=0;
  bool _dispensing=false,_paused=false,_incomplete=false,_complete=false,_endedAtFront=false;
  bool _seekFront=false;        // 이번 주입은 끝에서 리밋 찾기 허용
  bool _seeking=false;          // 목표 도착 후 전방 리밋 찾아 더 미는 중
  bool _seekMissed=false;       // 리밋을 FRONT_SEEK_MAX_MM 안에서 못 찾음
  long _seekFromStep=0; float _seekMm=0; float _frontPressMm=NAN;
  bool _dispenseLaunch=false;   // launchProfile()에 "주입 궤적"임을 알림(FRONT 끝 = 완료 허용)
  bool _overCurrent=false;
  float _currentMa=0;
  float _peakMa=0;          // 지난 [SERVO] 로그 이후 배터리 전류 최대값
  unsigned long _overCurrentMs=0;
};
extern Motor motor;
