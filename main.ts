//% weight=0 color=#0066CC icon="\uf2db" block="Matrix Micro"
namespace matrixmicro {
  pins.setPull(DigitalPin.P5, PinPullMode.PullUp)
  pins.setPull(DigitalPin.P11, PinPullMode.PullUp)

  //initial PCA9685 and set PWM frequency to 1526Hz
  function init() {
    pins.i2cWriteNumber(64, 16, NumberFormat.Int16BE, false)
    pins.i2cWriteNumber(64, 254 * 256 + 3, NumberFormat.Int16BE, false)
    pins.i2cWriteNumber(64, 0, NumberFormat.Int16BE, false)
  }
  init()

  /**
    DC Motor
  */
  //%block="Micro DC Motor %mpt |Speed %number"
  //%weight=99
  export function microMotor(mpt: Motor_port = 1, speed: number): void {
    let md = 0
    if (speed>100) {
      speed = 100
      md = 1
    } else if (speed <= 100 && speed > 0){
      md = 1
    } else if (speed < -100) {
      speed = 100
      md = 2
    } else if (speed >= -100 && speed < 0){
      speed = -speed
      md = 2
    } else if (speed == 0){
      md = 3
    }

    speed = Math.map(speed, 0, 100, 0, 4095)

    if (mpt == 1) {
      if (md == 1) {
        setpwm(11, speed)
        setpwm(13, 4095)
        setpwm(14, 0)
      }
      else if (md == 2) {
        setpwm(11, speed)
        setpwm(13, 0)
        setpwm(14, 4095)
      }
      else {
        setpwm(11, 0)
        setpwm(13, 0)
        setpwm(14, 0)
      }
    } else {
      if (md == 1) {
        setpwm(12, speed)
        setpwm(15, 4095)
        setpwm(16, 0)
      }
      else if (md == 2) {
        setpwm(12, speed)
        setpwm(15, 0)
        setpwm(16, 4095)
      }
      else {
        setpwm(12, 0)
        setpwm(15, 0)
        setpwm(16, 0)
      }
    }
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


  /**
    Digital Read
    Read digital port.
  */
  //% blockID="microDIGRead"  block="Micro Digital port %nd"
  //% blockGap=2 weight=96
  export function DRead(nd: Nd): number{
    let DP = 0
    switch (nd) {
      case 1:
        DP = pins.digitalReadPin(DigitalPin.P12)
        break;
      case 2:
        DP = pins.digitalReadPin(DigitalPin.P14)
        break;
    }
    return DP
  }
  export enum Nd {
    //% block="D1"
    DPort1 = 1,
    //% block="D2"
    DPort2 = 2,
  }

  /**
    Analog Read
  */
  //% blockID="microANGRead"  block="Micro Analog port %na"
  //% blockGap=2 weight=95
  export function ARead(na: Na): number{
    let AP = 0
    switch (na) {
      case 1:
        AP = pins.analogReadPin(AnalogPin.P1)
        break;
      case 2:
        AP = pins.analogReadPin(AnalogPin.P2)
        break;
    }
    return AP
  }
  export enum Na {
    //% block="A1"
    APort1 = 1,
    //% block="A2"
    APort2 = 2,
  }


  /**
    Micro Ultrasonic Sensor
  */
  //% blockId=ultrasonicSensor block="Micro Ultrasonic Sensor port %nu"
  //% weight=94
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
  export enum Nu {
    //% block="D1"
    UPort1 = 1,
    //% block="D2"
    UPort2 = 2,
  }


  /**
    RGB Led
  */
  //%block="Micro RGB Led port %seport R %number1 G %number2 B %number3"
  //%blockId="rgbled" weight=97
  export function rgb_led(seport: Led_port, r: number = 0, g: number = 0, b: number = 0): void {
    if (r > 100)r = 100
    if (r < 0)r = 0
    if (g > 100)g = 100
    if (g < 0)g = 0
    if (b > 100)b = 100
    if (b < 0)b = 0
    r = Math.map(r, 0, 100, 0, 2048)
    g = Math.map(g, 0, 100, 0, 2048)
    b = Math.map(b, 0, 100, 0, 2048)
    if (seport == 1) {
      setpwm(1, r)
      setpwm(2, g)
      setpwm(3, b)
    } else if (seport == 2) {
      setpwm(4, r)
      setpwm(5, g)
      setpwm(6, b)
    }
  }
  export enum Led_port {
    //% block="RGB1"
    S1 = 1,
    //% block="RGB2"
    S2 = 2,
  }

  function setpwm(Spin: number, Value: number) {
    let TM1 = Value % 256
    let TM2 = Value / 256
    let CH = (Spin - 1) * 4 + 8
    pins.i2cWriteNumber(64, CH * 256 + TM1, NumberFormat.Int16BE, false)
    pins.i2cWriteNumber(64, (CH + 1) * 256 + TM2, NumberFormat.Int16BE, false)
  }

    /**
    Servo movement
    choose one of the servo and set the angle.
  */
  //% blockID="microServo"  block="Micro RC Motor %ns|Angle %angle"
  //% blockGap=2 weight=98
  export function servo(ns: Nservo, angle: number): void{
    if (angle > 180)angle = 180
    if (angle < 0)angle = 0
    if (ns == 1) {
      pins.servoWritePin(AnalogPin.P8, angle)
    } else if (ns == 2) {
      pins.servoWritePin(AnalogPin.P16, angle)
    }
  }
  export enum Nservo {
    //% block="Servo1"
    SPort1 = 1,
    //% block="Servo2"
    Sport2 = 2,
  }
  namespace servos {
    //% fixedInstances
    export class Servo {
      private _minAngle: number;
      private _maxAngle: number;
      private _stopOnNeutral: boolean;

      constructor() {
        this._minAngle = 0;
        this._maxAngle = 180;
        this._stopOnNeutral = false;
      }
      private clampDegrees(degrees: number): number {
        degrees = degrees | 0;
        degrees = Math.clamp(this._minAngle, this._maxAngle, degrees);
        return degrees;
      }

      /**
        Set the servo angle
      */
      //% weight=100 help=servos/set-angle
      //% blockId=servoservosetangle block="set %servo angle to %degrees=protractorPicker °"
      //% degrees.defl=90
      //% servo.fieldEditor="gridpicker"
      //% servo.fieldOptions.width=220
      //% servo.fieldOptions.columns=2
      //% blockGap=8
      //% parts=microservo trackArgs=0
      //% group="Positional"
      setAngle(degrees: number) {
        degrees = this.clampDegrees(degrees);
        this.internalSetAngle(degrees);
      }

      protected internalSetAngle(angle: number): void {
      }
      /**
        Gets the minimum angle for the servo
      */
      public get minAngle() {
        return this._minAngle;
      }

      /**
        Gets the maximum angle for the servo
      */
      public get maxAngle() {
        return this._maxAngle;
      }

      /**
         Set the possible rotation range angles for the servo between 0 and 180
         @param minAngle the minimum angle from 0 to 90
         @param maxAngle the maximum angle from 90 to 180
      */
      //% help=servos/set-range
      //% blockId=servosetrange block="set %servo range from %minAngle to %maxAngle"
      //% minAngle.min=0 minAngle.max=90
      //% maxAngle.min=90 maxAngle.max=180 maxAngle.defl=180
      //% servo.fieldEditor="gridpicker"
      //% servo.fieldOptions.width=220
      //% servo.fieldOptions.columns=2
      //% parts=microservo trackArgs=0
      //% group="Configuration"
      //% blockGap=8
      public setRange(minAngle: number, maxAngle: number) {
        this._minAngle = Math.max(0, Math.min(90, minAngle | 0));
        this._maxAngle = Math.max(90, Math.min(180, maxAngle | 0));
      }

      /**
         Set a servo stop mode so it will stop when the rotation angle is in the neutral position, 90 degrees.
         @param on true to enable this mode
      */
      //% help=servos/set-stop-on-neutral
      //% blockId=servostoponneutral block="set %servo stop on neutral %enabled"
      //% enabled.shadow=toggleOnOff
      //% group="Configuration"
      //% blockGap=8
      //% servo.fieldEditor="gridpicker"
      //% servo.fieldOptions.width=220
      //% servo.fieldOptions.columns=2
      public setStopOnNeutral(enabled: boolean) {
        this._stopOnNeutral = enabled;
      }

      protected internalStop() { }
    }

    export class PinServo extends Servo {
      private _pin: PwmOnlyPin;
      constructor(pin: PwmOnlyPin) {
        super();
        this._pin = pin;
      }
      protected internalSetAngle(angle: number): void {
        this._pin.servoWrite(angle);
      }
      protected internalSetPulse(micros: number): void {
        this._pin.servoSetPulse(micros);
      }
      protected internalStop() {
        this._pin.digitalWrite(false);
      }
    }
  }
}
