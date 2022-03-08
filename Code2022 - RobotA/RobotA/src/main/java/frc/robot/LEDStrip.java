// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/** Add your docs here. */
public class LEDStrip {
    AddressableLED led;
    AddressableLEDBuffer ledBuffer;

    int tick;
    int tickM;
    public LEDStrip(int pwm, int length){
        this.led = new AddressableLED(pwm);
        this.ledBuffer = new AddressableLEDBuffer(length);
        this.led.setLength(ledBuffer.getLength());

        this.tick = -1;
        this.tickM = 0;
        for(int i = 0; i < this.ledBuffer.getLength(); i++){
            this.ledBuffer.setRGB(i, 50, 50, 50);
        }
        run();
    }

    private void run(){
        this.led.setData(this.ledBuffer);
        this.led.start();
        tick++;
    }

    //private void buildBufferData();

    public void blinkRGB(int delay, int r_value, int g_value, int b_value){
        if(this.tick % delay < delay/2){
            r_value = 0;
            g_value = 0;
            b_value = 0;
        }
        for(int i = 0; i < this.ledBuffer.getLength(); i++){
            this.ledBuffer.setRGB(i, r_value, g_value, b_value);
        }
        run();
    }

    public void staticRGB(int r_value, int g_value, int b_value){
        for(int i = 0; i < this.ledBuffer.getLength(); i++){
            this.ledBuffer.setRGB(i, r_value, g_value, b_value);
        }
        run();
    }
    public void movingRGB(int delay, int r_value, int g_value, int b_value, int lightPart, boolean reverse){//you can do better by adding darkPart
        //tickM = (tick/delay)%();
        // if (reverse){
        //     for(int i = (this.ledBuffer.getLength()/2)-1; i >(this.ledBuffer.getLength()/2)-1-tickM; i--){
        //         this.ledBuffer.setRGB(i, 0, 0, 0);
        //     }
        //     for(int i = (this.ledBuffer.getLength()/2)-1-tickM; i > ((this.ledBuffer.getLength()/2)-1-tickM)-lightPart; i--){
        //         this.ledBuffer.setRGB(i, r_value, g_value, b_value);
        //     }
        //     for(int i = ((this.ledBuffer.getLength()/2)-1-tickM)-lightPart; i >= 0; i--){
        //         this.ledBuffer.setRGB(i, 0, 0, 0);
        //     }
        //     //
        //     for(int i = this.ledBuffer.getLength()/2; i < (this.ledBuffer.getLength()/2)+tickM; i++){
        //         this.ledBuffer.setRGB(i, 0, 0, 0);
        //     }
        //     for(int i = (this.ledBuffer.getLength()/2)+tickM; i < (this.ledBuffer.getLength()/2)+tickM + lightPart; i++){
        //         this.ledBuffer.setRGB(i, r_value, g_value, b_value);
        //     }
        //     for(int i = (this.ledBuffer.getLength()/2) + tickM + lightPart; i < this.ledBuffer.getLength(); i++){
        //         this.ledBuffer.setRGB(i, 0, 0, 0);
        //     }

        //     return;
        // }
        // for(int i = 0; i < tickM; i++){
        //     this.ledBuffer.setRGB(i, 0, 0, 0);
        // }
        // for(int i = tickM; i < tickM + lightPart; i++){
        //     this.ledBuffer.setRGB(i, r_value, g_value, b_value);
        // }
        // for(int i = tickM + lightPart; i < this.ledBuffer.getLength()/2; i++){
        //     this.ledBuffer.setRGB(i, 0, 0, 0);
        // }
        // //
        // for(int i = this.ledBuffer.getLength()-1; i > this.ledBuffer.getLength()-1-tickM; i--){
        //     this.ledBuffer.setRGB(i, 0, 0, 0);
        // }
        // for(int i = this.ledBuffer.getLength()-2-tickM; i > (this.ledBuffer.getLength()-1-tickM) - lightPart; i--){
        //     this.ledBuffer.setRGB(i, r_value, g_value, b_value);
        // }
        // for(int i = (this.ledBuffer.getLength()-1-tickM) - lightPart; i >= this.ledBuffer.getLength()/2; i--){
        //     this.ledBuffer.setRGB(i, 0, 0, 0);
        // }

        run();

    }
}
