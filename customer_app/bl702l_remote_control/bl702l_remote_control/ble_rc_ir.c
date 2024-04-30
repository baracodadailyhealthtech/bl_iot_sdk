#include "ble_rc_ir.h"
#include "bl_ir.h"
#include "bl_pwm_ir.h"
#include "bl_timer.h"
#include "bl_irq.h"
#include "bl_os_port.h"

//#define USE_PWM_IR

#if !defined(USE_PWM_IR)
//interrupt callback after sending a pulse
void bl_ir_swm_tx_done_callback(void)
{
    //printf("bl_ir_swm_tx_done_callback\r\n");
}
#endif

void ble_rc_ir_tx_init(void)
{
#if defined(USE_PWM_IR)
    //frequency: 37.7KHz; duty cycle: 1/3
    bl_pwm_ir_tx_cfg(37700, 0.3333);
    
    //ir output on gpio22
    bl_pwm_ir_tx_pin_cfg(22);
#else
    //frequency: 37.7KHz; duty cycle: 1/3
    bl_ir_swm_tx_cfg(37700, 0.3333);
    
    //gpio22: led0; gpio18: led1, not mounted on BL702L_DVK
    bl_ir_led_drv_cfg(1, 0);
#endif
    
    //hardware timer
    bl_timer_init();
}

void ble_rc_ir_tx_demo(void)
{
    uint16_t ir_tx[] = {2, 2, 4, 2, 6};
    
#if defined(USE_PWM_IR)
    bl_pwm_ir_tx(ir_tx, sizeof(ir_tx)/2);
#else
    bl_ir_swm_tx(1, ir_tx, sizeof(ir_tx)/2);
    //while(bl_ir_swm_tx_busy());
#endif
}

void ble_rc_ir_tx_repeat_demo(void)
{
#if defined(USE_PWM_IR)
    uint16_t ir_tx[] = {340, 85, 21};
    bl_pwm_ir_tx(ir_tx, sizeof(ir_tx)/2);
#else
    uint16_t ir_tx[] = {340, 84, 20};  // tx data should be multiple of k
    bl_ir_swm_tx(2, ir_tx, sizeof(ir_tx)/2);  // here set k to 2, so that max carrier number can be 256*2
    //while(bl_ir_swm_tx_busy());
#endif
}


/*
  Let D = delay_us (argement of ble_rc_ir_timer_start, the delay of next IR transmission), T1 = IR_THRESHOLD_1, T2 = IR_THRESHOLD_2.
  
  Step 1: If D > T1, a software timer will be started, and expired at T1.
  Step 2: In the software timer callback, a hardware timer will be started, and expired at T2.
  Step 3: In the hardware timer interrupt service routine, wait for the expiration of D, and transmit.
  
  Note: PDS (Power Down Sleep) is allowed before software timer expiration, but not allowed after software timer expiration.
*/
#define IR_THRESHOLD_1       15  // ms, use software timer, sleep allowed
#define IR_THRESHOLD_2       5000  // us, use hardware timer, sleep not allowed


static uint32_t start_us = 0;
static uint32_t target_us = 0;
static void (*ir_cb)(void) = NULL;
static int ir_busy = 0;
static void *sw_timer = NULL;


static void hw_timer_cb(void)
{
    uint32_t current_us = bl_timer_now_us();
    
    if((int32_t)(target_us - current_us) >= 0){
        bl_timer_delay_us(target_us - current_us);
        
        ir_busy = 0;
        if(ir_cb){
            ir_cb();
        }
    }else{
        printf("assert: hw_timer_cb, target_us: %lu, current_us: %lu\r\n", target_us, current_us);
    }
}

static void sw_timer_cb(void *timer)
{
    int mstatus = bl_irq_save();
    uint32_t current_us = bl_timer_now_us();
    
    if((int32_t)(target_us - IR_THRESHOLD_2 - current_us) >= 0){
        ir_busy = 1;
        bl_timer_start(0, bl_timer_get_current_time() + target_us - IR_THRESHOLD_2 - current_us, hw_timer_cb);
    }else{
        printf("assert: sw_timer_cb, target_us: %lu, current_us: %lu\r\n", target_us, current_us);
    }
    
    bl_irq_restore(mstatus);
}


void ble_rc_ir_timer_start(uint32_t delay_us, void (*cb)(void))
{
    int mstatus = bl_irq_save();
    
    start_us = bl_timer_now_us();
    target_us = start_us + delay_us;
    ir_cb = cb;
    ir_busy = 0;
    
    if(delay_us/1000 > IR_THRESHOLD_1){
        if(sw_timer){
            bl_os_timer_change_period(sw_timer, delay_us/1000 - IR_THRESHOLD_1);
        }else{
            sw_timer = bl_os_timer_create("ir", delay_us/1000 - IR_THRESHOLD_1, sw_timer_cb, 0);
            bl_os_timer_start(sw_timer);
        }
    }else if(delay_us > IR_THRESHOLD_2){
        ir_busy = 1;
        bl_timer_start(0, bl_timer_get_current_time() + delay_us - IR_THRESHOLD_2, hw_timer_cb);
    }else{
        if(delay_us != 0){
            bl_timer_delay_us(delay_us);
        }
        if(cb){
            cb();
        }
    }
    
    bl_irq_restore(mstatus);
}

void ble_rc_ir_timer_stop(void)
{
    bl_timer_stop(0);
    
    if(sw_timer){
        bl_os_timer_stop(sw_timer);
    }
    
    ir_busy = 0;
}

int ble_rc_ir_is_busy(void)
{
#if !defined(USE_PWM_IR)
    if(bl_ir_swm_tx_busy()){
        return 1;
    }
#endif
    
    return ir_busy;
}

void ble_rc_ir_tx_demo_with_timer(void)
{
    uint32_t t1, t2;
    uint32_t tx_cost;
    uint32_t repeat_delay = 110*1000;
    
    t1 = bl_timer_now_us();
    
    ble_rc_ir_tx_demo();
    
    t2 = bl_timer_now_us();
    
    tx_cost = t2 - t1;
    
    if(repeat_delay >= tx_cost){
        ble_rc_ir_timer_start(repeat_delay - tx_cost, ble_rc_ir_tx_repeat_demo_with_timer);
    }else{
        printf("assert: ble_rc_ir_tx_demo_with_timer, repeat_delay: %lu, tx_cost: %lu\r\n", repeat_delay, tx_cost);
    }
}

void ble_rc_ir_tx_repeat_demo_with_timer(void)
{
    uint32_t t1, t2;
    uint32_t tx_cost;
    uint32_t repeat_delay = 110*1000;
    
    t1 = bl_timer_now_us();
    
    ble_rc_ir_tx_repeat_demo();
    printf("repeat code\r\n");
    
    t2 = bl_timer_now_us();
    
    tx_cost = t2 - t1;
    
    if(repeat_delay >= tx_cost){
        ble_rc_ir_timer_start(repeat_delay - tx_cost, ble_rc_ir_tx_repeat_demo_with_timer);
    }else{
        printf("assert: ble_rc_ir_tx_repeat_demo_with_timer, repeat_delay: %lu, tx_cost: %lu\r\n", repeat_delay, tx_cost);
    }
}
