#ifndef HALL_SWITCH_H_
#define HALL_SWITCH_H_

class HallSwitch {
 public:
  HallSwitch(int power_pin, int state_pin);
  void init();
  bool isInitialized() const;
  void setPowerState(bool power_state);
  bool isTriggered() const;

 private:
   const int power_pin_;
   const int state_pin_;
   bool is_initialized_;
};

#endif
