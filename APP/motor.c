#include "motor.h"

static bool GPIOPinIsHigh(const GPIO_Pin_t *pin)
{
    return (DL_GPIO_readPins(pin->port, pin->pin) != 0U);
}

static bool GPIOPinIsTriggered(const GPIO_Pin_t *pin, GPIO_Regs *interrupt_port,
    uint32_t interrupt_pin)
{
    return ((pin->port == interrupt_port) && ((interrupt_pin & pin->pin) != 0U));
}

static void GPIOPinWrite(const GPIO_Pin_t *pin, bool high)
{
    if (high) {
        DL_GPIO_setPins(pin->port, pin->pin);
    } else {
        DL_GPIO_clearPins(pin->port, pin->pin);
    }
}

static L298N_Channel_t *L298N_GetChannel(L298N_t *handle, uint16_t channel)
{
    if (handle == NULL) {
        return NULL;
    }

    switch (channel) {
    case 1:
        return &handle->motor1;
    case 2:
        return &handle->motor2;
    default:
        return NULL;
    }
}

static uint32_t AbsClampPWMValue(int32_t pwm_value, uint32_t max_value)
{
    uint32_t abs_value;

    if (pwm_value < 0) {
        abs_value = (uint32_t) (-(pwm_value + 1)) + 1U;
    } else {
        abs_value = (uint32_t) pwm_value;
    }

    return (abs_value > max_value) ? max_value : abs_value;
}

void L298N_Init(L298N_t*handle)
{
    if ((handle == NULL) || (handle->pwm_timer == NULL)) {
        return;
    }

    L298N_SetPWMValue(handle, 1, 0);
    L298N_SetPWMValue(handle, 2, 0);
    DL_Timer_startCounter(handle->pwm_timer);
}

//开启该编码器对应的外部中断
void  EncoderInit(Encoder_t*handle)
{
    if (handle == NULL) {
        return;
    }

    handle->encoder_value = 0;

    if (handle->chb.port == handle->cha.port) {
        uint32_t pins = handle->cha.pin | handle->chb.pin;

        DL_GPIO_clearInterruptStatus(handle->cha.port, pins);
        DL_GPIO_enableInterrupt(handle->cha.port, pins);
    } else {
        DL_GPIO_clearInterruptStatus(handle->cha.port, handle->cha.pin);
        DL_GPIO_enableInterrupt(handle->cha.port, handle->cha.pin);
        DL_GPIO_clearInterruptStatus(handle->chb.port, handle->chb.pin);
        DL_GPIO_enableInterrupt(handle->chb.port, handle->chb.pin);
    }

}

//自动根据pwm_value的正负确认方向控制引脚的电平
void L298N_SetPWMValue(L298N_t*handle,uint16_t channel,int32_t pwm_value)
{
    L298N_Channel_t *motor = L298N_GetChannel(handle, channel);
    uint32_t max_pwm_value;
    uint32_t compare_value;
    bool forward;

    if ((handle == NULL) || (motor == NULL) || (handle->pwm_timer == NULL)) {
        return;
    }

    max_pwm_value = DL_Timer_getLoadValue(handle->pwm_timer);
    compare_value = AbsClampPWMValue(pwm_value, max_pwm_value);

    if (compare_value == 0U) {
        GPIOPinWrite(&motor->in1, false);
        GPIOPinWrite(&motor->in2, false);
    } else {
        forward = (pwm_value > 0);
        if (motor->invert_direction) {
            forward = !forward;
        }

        GPIOPinWrite(&motor->in1, forward);
        GPIOPinWrite(&motor->in2, !forward);
    }

    motor->duty_permille =
        (max_pwm_value == 0U) ? 0U : (uint16_t) ((compare_value * 1000U) / max_pwm_value);
    DL_Timer_setCaptureCompareValue(handle->pwm_timer, compare_value, motor->pwm_cc_index);
}

//判定中断触发的引脚是否属于当前编码器，如果是的话AB通道双边沿4倍速计数，并返回true
bool EncoderUpdate(Encoder_t *handle, GPIO_Regs *interrupt_port, uint32_t interrupt_pin)
{
    bool updated = false;

    if ((handle == NULL) || (interrupt_port == NULL) || (interrupt_pin == 0U)) {
        return false;
    }

    if (GPIOPinIsTriggered(&handle->cha, interrupt_port, interrupt_pin)) {
        bool a_level = GPIOPinIsHigh(&handle->cha);
        bool b_level = GPIOPinIsHigh(&handle->chb);

        if (a_level == b_level) {
            handle->encoder_value++;
        } else {
            handle->encoder_value--;
        }
        updated = true;
    }

    if (GPIOPinIsTriggered(&handle->chb, interrupt_port, interrupt_pin)) {
        bool a_level = GPIOPinIsHigh(&handle->cha);
        bool b_level = GPIOPinIsHigh(&handle->chb);

        if (a_level != b_level) {
            handle->encoder_value++;
        } else {
            handle->encoder_value--;
        }
        updated = true;
    }

    return updated;
}
