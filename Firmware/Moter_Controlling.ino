void left_forward(byte m_speed){
  digitalWrite(STBY, HIGH);

  digitalWrite(LEFTM_DIRECT1, HIGH);
  digitalWrite(LEFTM_DIRECT2, LOW);
  analogWrite(LEFTM_PWM, m_speed);
  Serial.print("forward : ");
  Serial.println(m_speed);
}

void right_forward(byte m_speed){
  digitalWrite(STBY, HIGH);

  digitalWrite(RIGHTM_DIRECT1, HIGH);
  digitalWrite(RIGHTM_DIRECT2, LOW);
  analogWrite(RIGHTM_PWM, m_speed);
}


void left_backward(byte m_speed){
  digitalWrite(STBY, HIGH);

  digitalWrite(LEFTM_DIRECT1, LOW);
  digitalWrite(LEFTM_DIRECT2, HIGH);
  analogWrite(LEFTM_PWM, m_speed);
  Serial.print("Backward : ");
  Serial.println(m_speed);
}

void right_backward(byte m_speed){
  digitalWrite(STBY, HIGH);

  digitalWrite(RIGHTM_DIRECT1, LOW);
  digitalWrite(RIGHTM_DIRECT2, HIGH);
  analogWrite(RIGHTM_PWM, m_speed);
}
