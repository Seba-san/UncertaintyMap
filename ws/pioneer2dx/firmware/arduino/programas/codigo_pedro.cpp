#int_timer1 HIGH // TIMER1
void contmotor(void) {

    acc = Lee_ADC(0) * (5.0 / 1023.0);
    // acc=(acc-2.5)/0.066;
    acc = (acc) / 0.13;
    QEIval.words[0] = qei_get_count();
    posicion = QEIval.total;
    distancia = posicion - count;
    count = QEIval.total;
    VelMed = (float)distancia / 2400.0;
    VelMed = (float)VelMed / (Ts);
    // VelMed=(float)VelMed;
    if (dir < 3) {
        switch (pidselect) {
        case 0:
            break;
        case 1:
            PID_vel();
            break;
        case 2:
            PID_pos();
            PID_vel();
            break;
        default:
            ERROR();
            break;
        }
    }
}

void PID_vel(void) {
    ekv = VelRef - VelMed; // error
    Proporcional = Kpv * (ekv);
    Derivativo = (Kdv * (ekv - ekv1)) / Ts;
    ucontrolv = ucontrolv1 + Proporcional + Derivativo;
    ucontrolv2 = ucontrolv1;
    ucontrolv1 = ucontrolv;
    ekv2 = ekv1;
    ekv1 = ekv;

    if (ucontrolv < 0) {
        sig = 1;
    }
    else {
        sig = 2;
    }
    if (sig != siga) {
        siga = sig;
        if (ucontrolv < 0) {
            dir = 1;
            delay_us(5);
            Giro_MOTOR('d');
        }
        else {
            dir = 2;
            delay_us(5);
            Giro_MOTOR('i');
        }
    }

    if (ucontrolv > 499)
        ucontrolv = 499;
    if (ucontrolv < -499)
        ucontrolv = -499;

    duty = (int)fabs(ucontrolv);
    SET_PWM_DUTY(1, duty);
}
void PID_pos(void) {
    ekp = posiciond - posicion; // error
    Proporcional = Kpp * (ekp);
    Integral = Integral + Kip * Ts * (ekp);
    Derivativo = (Kdp * (ekp - ekp1) / Ts);
    ucontrolp = Proporcional + Derivativo + Integral;
    if (ucontrolp > 4)
        ucontrolp = 4;
    if (ucontrolp < -4)
        ucontrolp = -4;
    ucontrolp2 = ucontrolp1;
    ucontrolp1 = ucontrolp;
    ekp2 = ekp1;
    ekp1 = ekp;
    VelRef = ucontrolp;
}
