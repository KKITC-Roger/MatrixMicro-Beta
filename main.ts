//% weight=0 color=#0066CC icon="\uf2db" block="Micro"
namespace matrixmicro {

  pins.setPull(DigitalPin.P5, PinPullMode.PullUp)
  pins.setPull(DigitalPin.P11, PinPullMode.PullUp)

  let K = 4096 / 20
	let StartBit = 0.5 * K
	let FullScaleBit = 1.94 * K

  function init() {
  	pins.i2cWriteNumber(64, 16, NumberFormat.Int16BE, false)
  	pins.i2cWriteNumber(64, 254 * 256 + 123, NumberFormat.Int16BE, false)
  	pins.i2cWriteNumber(64, 0, NumberFormat.Int16BE, false)
  }

  init()
  /**
  * Button menu
  */
  export enum Nbtn{
    //% block="Button 1"
    BTN1 = 1,
    //% block="Button 2"
    BTN2 = 2,
  }
  /**
  * Button state
  * Detect the button is pressed or not
  */
  //% blockID="microButtonState"  block="Micro %nb| press?"
  //% blockGap=2 weight=97
  export function button(nb: Nbtn): number{
    let PUpin = 0
    switch (nb) {
      case 1: PUpin = pins.digitalReadPin(DigitalPin.P5)
        break;
      case 2: PUpin = pins.digitalReadPin(DigitalPin.P11)
        break;
    }
    return PUpin
  }

  export enum Nservo{
    //% block="Servo1"
    SPort1 = 1,
    //% block="Servo2"
    Sport2 = 2,
  }
  /**
  * Servo movement
  * choose one of the servo and set the angle.
  */
  //% blockID="microServo"  block="Micro RC Motor %ns|Angle %angle"
  //% blockGap=2 weight=98
  export function servo(ns: Nservo, angle: number): void{
    if(angle>180)angle = 180
    if(angle<0)angle = 0
    let TS1 = (angle / 180 * FullScaleBit + StartBit) % 256
  	let TS2 = (angle / 180 * FullScaleBit + StartBit) / 256
  	let CH = (ns - 1) * 4 + 8
  	pins.i2cWriteNumber(64, CH * 256 + TS1, NumberFormat.Int16BE, false)
  	pins.i2cWriteNumber(64, (CH + 1) * 256 + TS2, NumberFormat.Int16BE, false)
  }



  export enum Motor_port {
	  //% block="M1"
	  M1 = 1,
    //% block="M2"
    M2 = 2
	}
  export enum Motor_state {
	  //% block="Forward"
	  A1 = 1,
    //% block="Reverse"
    A2 = 2,
    //% block="Stop"
    A3 = 3
	}
  //%block="Micro DC Motor %mpt %apt|Speed %number"
	export function microMotor(mpt: Motor_port = 1, apt: Motor_state = 1, speed: number): void {
		if (speed > 100)speed = 100
		if (speed < 0)speed = 0
    speed = Math.map(speed, 0, 100, 0, 4095)

    if (mpt == 1) {
      if (apt == 1) {
        motor(11, speed)
        motor(13, 4095)
        motor(14, 0)
      }
      else if (apt == 2) {
        motor(11, speed)
        motor(13, 0)
        motor(14, 4095)
      }
      else {
        motor(11, 0)
        motor(13, 0)
        motor(14, 0)
      }
    } else {
      if (apt == 1) {
        motor(12, speed)
        motor(15, 4095)
        motor(16, 0)
      }
      else if (apt == 2) {
        motor(12, speed)
        motor(15, 0)
        motor(16, 4095)
      }
      else {
        motor(12, 0)
        motor(15, 0)
        motor(16, 0)
      }
    }
  }
  function motor(Spin: number, Speed: number) {
		let TM1 = Speed % 256
		let TM2 = Speed / 256
		let CH = (Spin - 1) * 4 + 8
		pins.i2cWriteNumber(64, CH * 256 + TM1, NumberFormat.Int16BE, false)
		pins.i2cWriteNumber(64, (CH + 1) * 256 + TM2, NumberFormat.Int16BE, false)
	}


  export enum Nd{
    //% block="D1"
    DPort1 = 1,
    //% block="D2"
    DPort2 = 2,
  }
  /**
  * Digital Read
  * Read digital port.
  */
  //% blockID="microDIGRead"  block="Micro Digital port %nd"
  //% blockGap=2 weight=96
  export function DRead(nd: Nd): number{
    let DP = 0
    switch(nd){
      case 1:
        DP = pins.digitalReadPin(DigitalPin.P12)
        break;
      case 2:
        DP = pins.digitalReadPin(DigitalPin.P14)
        break;
      }
    return DP
  }

  export enum Na{
    //% block="A1"
    APort1 = 1,
    //% block="A2"
    APort2 = 2,
  }
  /**
  * Analog Read
  * Read analog port.
  */
  //% blockID="microANGRead"  block="Micro Analog port %na"
  //% blockGap=2 weight=96
  export function ARead(na: Na): number{
    let AP = 0
    switch(na){
      case 1:
        AP = pins.analogReadPin(AnalogPin.P0)
        break;
      case 2:
        AP = pins.analogReadPin(AnalogPin.P2)
        break;
      }
    return AP
  }
  export enum Nu{
    //% block="D1"
    UPort1 = 1,
    //% block="D2"
    UPort2 = 2,
  }
  /**
  * Micro Ultrasonic Sensor
  */
  //% blockId=ultrasonicSensor block="Micro Ultrasonic Sensor port %nu"
  //% weight=10
  export function microUltrasonicSensor(nu: Nu): number {
    let pinT = DigitalPin.P0
    let pinE = DigitalPin.P1
    switch (nu) {
      case 1:
        pinT = DigitalPin.P12
        pinE = DigitalPin.P13
        break;
      case 2:
        pinT = DigitalPin.P14
        pinE = DigitalPin.P15
        break;
    }
    // send pulse
    pins.setPull(pinT, PinPullMode.PullNone);
    pins.digitalWritePin(pinT, 0);
    control.waitMicros(2);
    pins.digitalWritePin(pinT, 1);
    control.waitMicros(10);
    pins.digitalWritePin(pinT, 0);

    // read pulse
    let d = pins.pulseIn(pinE, PulseValue.High, 23000);  // 8 / 340 =
    return d * 0.017;
  }

}
