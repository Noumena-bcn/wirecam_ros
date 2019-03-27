//
// Copyright (c) 2019 Noumena - ROMI
// Created by Noumena on 24/03/19.
//

class RECEIVER_X8R{
  private:
  double ros_msg[];
  public:    
    double PIN_CH1_PIN2 = 2;
    double PIN_CH2_PIN3 = 3;
    double PIN_CH3_PIN4 = 4;
    double PIN_CH4_PIN5 = 5;
    double PIN_CH5_PIN6 = 6;
    double PIN_CH6_PIN7 = 7;
    double PIN_CH7_PIN8 = 8;
    double PIN_CH8_PIN9 = 8;
    double PWM_CH1;
    double PWM_CH2;
    double PWM_CH3;
    double PWM_CH4;
    double PWM_CH5;
    double PWM_CH6;
    double PWM_CH7;
    double PWM_CH8;
    double DATA_CH1;
    double DATA_CH2;
    double DATA_CH3;
    double DATA_CH4;
    double DATA_CH5;
    double DATA_CH6;
    double DATA_CH7;
    double DATA_CH8;

    void receiver_setup(){
      pinMode(PIN_CH1_PIN2, INPUT);
      pinMode(PIN_CH2_PIN3, INPUT);
      pinMode(PIN_CH3_PIN4, INPUT);
      pinMode(PIN_CH4_PIN5, INPUT);
      pinMode(PIN_CH5_PIN6, INPUT);
      pinMode(PIN_CH6_PIN7, INPUT);
      pinMode(PIN_CH8_PIN9, INPUT);
    }

    void receiver_loop(){
      DATA_CH1 = pulseIn(PIN_CH1_PIN2, HIGH, 25000);
      DATA_CH2 = pulseIn(PIN_CH2_PIN3, HIGH, 25000);
      DATA_CH3 = pulseIn(PIN_CH3_PIN4, HIGH, 25000);
      DATA_CH4 = pulseIn(PIN_CH4_PIN5, HIGH, 25000);
      DATA_CH5 = pulseIn(PIN_CH5_PIN6, HIGH, 25000);
      DATA_CH6 = pulseIn(PIN_CH6_PIN7, HIGH, 25000);
      DATA_CH7 = pulseIn(PIN_CH7_PIN8, HIGH, 25000);
      DATA_CH8 = pulseIn(PIN_CH8_PIN9, HIGH, 25000);
    }
};
