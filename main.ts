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
    //% block="A1"
    UPort3 = 3,
    //% block="A2"
    UPort4 = 4,
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
      case 3:
        pinT = DigitalPin.P0
        pinE = DigitalPin.P1
        break;
      case 4:
        pinT = DigitalPin.P2
        pinE = DigitalPin.P3
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
    return d * 5 / 3 / 58;
  }

  export class Strip {
      buf: Buffer;
      pin: DigitalPin;
      // TODO: encode as bytes instead of 32bit
      brightness: number;
      start: number; // start offset in LED strip
      _length: number; // number of LEDs
      _mode: NeoPixelMode;
      _matrixWidth: number; // number of leds in a matrix - if any

      /**
       * Shows all LEDs to a given color (range 0-255 for r, g, b).
       * @param rgb RGB color of the LED
       */
      //% blockId="neopixel_set_strip_color" block="%strip|show color %rgb=neopixel_colors"
      //% weight=85 blockGap=8
      //% parts="neopixel"
      showColor(rgb: number) {
          rgb = rgb >> 0;
          this.setAllRGB(rgb);
          this.show();
      }


      /**
       * Send all the changes to the strip.
       */
      //% blockId="neopixel_show" block="%strip|show" blockGap=8
      //% weight=79
      //% parts="neopixel"
      show() {
          ws2812b.sendBuffer(this.buf, this.pin);
      }

      /**
       * Turn off all LEDs.
       * You need to call ``show`` to make the changes visible.
       */
      //% blockId="neopixel_clear" block="%strip|clear"
      //% weight=76
      //% parts="neopixel"
      clear(): void {
          const stride = this._mode === NeoPixelMode.RGB;
          this.buf.fill(0, this.start * stride, this._length * stride);
      }

      /**
       * Gets the number of pixels declared on the strip
       */
      //% blockId="neopixel_length" block="%strip|length" blockGap=8
      //% weight=60 advanced=true
      length() {
          return this._length;
      }

      /**
       * Set the brightness of the strip. This flag only applies to future operation.
       * @param brightness a measure of LED brightness in 0-255. eg: 255
       */
      //% blockId="neopixel_set_brightness" block="%strip|set brightness %brightness" blockGap=8
      //% weight=59
      //% parts="neopixel" advanced=true
      setBrightness(brightness: number): void {
          this.brightness = brightness & 0xff;
      }

      /**
      * Apply brightness to current colors using a quadratic easing function.
      **/
      //% blockId="neopixel_each_brightness" block="%strip|ease brightness" blockGap=8
      //% weight=58
      //% parts="neopixel" advanced=true
      easeBrightness(): void {
          const stride = this._mode === NeoPixelMode.RGB;
          const br = this.brightness;
          const buf = this.buf;
          const end = this.start + this._length;
          const mid = Math.idiv(this._length, 2);
          for (let i = this.start; i < end; ++i) {
              const k = i - this.start;
              const ledoffset = i * stride;
              const br = k > mid
                  ? Math.idiv(255 * (this._length - 1 - k) * (this._length - 1 - k), (mid * mid))
                  : Math.idiv(255 * k * k, (mid * mid));
              const r = (buf[ledoffset + 0] * br) >> 8; buf[ledoffset + 0] = r;
              const g = (buf[ledoffset + 1] * br) >> 8; buf[ledoffset + 1] = g;
              const b = (buf[ledoffset + 2] * br) >> 8; buf[ledoffset + 2] = b;
              if (stride == 4) {
                  const w = (buf[ledoffset + 3] * br) >> 8; buf[ledoffset + 3] = w;
              }
          }
      }

      /**
       * Create a range of LEDs.
       * @param start offset in the LED strip to start the range
       * @param length number of LEDs in the range. eg: 4
       */
      //% weight=89
      //% blockId="neopixel_range" block="%strip|range from %start|with %length|leds"
      //% parts="neopixel"
      //% blockSetVariable=range
      range(start: number, length: number): Strip {
          start = start >> 0;
          length = length >> 0;
          let strip = new Strip();
          strip.buf = this.buf;
          strip.pin = this.pin;
          strip.brightness = this.brightness;
          strip.start = this.start + Math.clamp(0, this._length - 1, start);
          strip._length = Math.clamp(0, this._length - (strip.start - this.start), length);
          strip._matrixWidth = 0;
          strip._mode = this._mode;
          return strip;
      }

      /**
       * Shift LEDs forward and clear with zeros.
       * You need to call ``show`` to make the changes visible.
       * @param offset number of pixels to shift forward, eg: 1
       */
      //% blockId="neopixel_shift" block="%strip|shift pixels by %offset" blockGap=8
      //% weight=40
      //% parts="neopixel"
      shift(offset: number = 1): void {
          offset = offset >> 0;
          const stride = this._mode === NeoPixelMode.RGB;
          this.buf.shift(-offset * stride, this.start * stride, this._length * stride)
      }

      /**
       * Rotate LEDs forward.
       * You need to call ``show`` to make the changes visible.
       * @param offset number of pixels to rotate forward, eg: 1
       */
      //% blockId="neopixel_rotate" block="%strip|rotate pixels by %offset" blockGap=8
      //% weight=39
      //% parts="neopixel"
      rotate(offset: number = 1): void {
          offset = offset >> 0;
          const stride = this._mode === NeoPixelMode.RGB;
          this.buf.rotate(-offset * stride, this.start * stride, this._length * stride)
      }

      /**
       * Set the pin where the neopixel is connected, defaults to P0.
       */
      //% weight=10
      //% parts="neopixel" advanced=true
      setPin(pin: DigitalPin): void {
          this.pin = DigitalPin.P8;
          pins.digitalWritePin(this.pin, 0);
          // don't yield to avoid races on initialization
      }

      /**
       * Estimates the electrical current (mA) consumed by the current light configuration.
       */
      //% weight=9 blockId=neopixel_power block="%strip|power (mA)"
      //% advanced=true
      power(): number {
          const stride = this._mode === NeoPixelMode.RGB;
          const end = this.start + this._length;
          let p = 0;
          for (let i = this.start; i < end; ++i) {
              const ledoffset = i * stride;
              for (let j = 0; j < stride; ++j) {
                  p += this.buf[i + j];
              }
          }
          return Math.idiv(this.length() * 7, 10) /* 0.7mA per neopixel */
              + Math.idiv(p * 480, 10000); /* rought approximation */
      }

      private setBufferRGB(offset: number, red: number, green: number, blue: number): void {
          if (this._mode === NeoPixelMode.RGB_RGB) {
              this.buf[offset + 0] = red;
              this.buf[offset + 1] = green;
          } else {
              this.buf[offset + 0] = green;
              this.buf[offset + 1] = red;
          }
          this.buf[offset + 2] = blue;
      }

      private setAllRGB(rgb: number) {
          let red = unpackR(rgb);
          let green = unpackG(rgb);
          let blue = unpackB(rgb);

          const br = this.brightness;
          if (br < 255) {
              red = (red * br) >> 8;
              green = (green * br) >> 8;
              blue = (blue * br) >> 8;
          }
          const end = this.start + this._length;
          const stride = this._mode === NeoPixelMode.RGB;
          for (let i = this.start; i < end; ++i) {
              this.setBufferRGB(i * stride, red, green, blue)
          }
      }
      private setAllW(white: number) {
          if (this._mode !== NeoPixelMode.RGBW)
              return;

          let br = this.brightness;
          if (br < 255) {
              white = (white * br) >> 8;
          }
          let buf = this.buf;
          let end = this.start + this._length;
          for (let i = this.start; i < end; ++i) {
              let ledoffset = i * 4;
              buf[ledoffset + 3] = white;
          }
      }

  /**
   * Create a new NeoPixel driver for `numleds` LEDs.
   * @param pin the pin where the neopixel is connected.
   * @param numleds number of leds in the strip, eg: 24,30,60,64
   */
  //% blockId="neopixel_create" block="NeoPixel"
  //% weight=90 blockGap=2
  //% parts="neopixel"
  //% trackArgs=0,2
  //% blockSetVariable=strip
  private function create(): Strip {
      let strip = new Strip();
      let stride = mode === NeoPixelMode.RGB;
      strip.buf = pins.createBuffer(2 * stride);
      strip.start = 0;
      strip._length = 2;
      strip._mode = NeoPixelMode.RGB;
      strip._matrixWidth = 0;
      strip.setBrightness(128)
      strip.setPin(DigitalPin.P8)
      return strip;
  }

  /**
   * Converts red, green, blue channels into a RGB color
   * @param red value of the red channel between 0 and 255. eg: 255
   * @param green value of the green channel between 0 and 255. eg: 255
   * @param blue value of the blue channel between 0 and 255. eg: 255
   */
  //% weight=1
  //% blockId="neopixel_rgb" block="red %red|green %green|blue %blue"
  //% advanced=true
  export function rgb(red: number, green: number, blue: number): number {
      return packRGB(red, green, blue);
  }

  /**
   * Gets the RGB value of a known color
  */
  //% weight=2 blockGap=8
  //% blockId="neopixel_colors" block="%color"
  //% advanced=true
  export function colors(color: NeoPixelColors): number {
      return color;
  }

  function packRGB(a: number, b: number, c: number): number {
      return ((a & 0xFF) << 16) | ((b & 0xFF) << 8) | (c & 0xFF);
  }
  function unpackR(rgb: number): number {
      let r = (rgb >> 16) & 0xFF;
      return r;
  }
  function unpackG(rgb: number): number {
      let g = (rgb >> 8) & 0xFF;
      return g;
  }
  function unpackB(rgb: number): number {
      let b = (rgb) & 0xFF;
      return b;
  }

  /**
   * Converts a hue saturation luminosity value into a RGB color
   * @param h hue from 0 to 360
   * @param s saturation from 0 to 99
   * @param l luminosity from 0 to 99
   */
  //% blockId=neopixelHSL block="hue %h|saturation %s|luminosity %l"
  export function hsl(h: number, s: number, l: number): number {
      h = Math.round(h);
      s = Math.round(s);
      l = Math.round(l);

      h = h % 360;
      s = Math.clamp(0, 99, s);
      l = Math.clamp(0, 99, l);
      let c = Math.idiv((((100 - Math.abs(2 * l - 100)) * s) << 8), 10000); //chroma, [0,255]
      let h1 = Math.idiv(h, 60);//[0,6]
      let h2 = Math.idiv((h - h1 * 60) * 256, 60);//[0,255]
      let temp = Math.abs((((h1 % 2) << 8) + h2) - 256);
      let x = (c * (256 - (temp))) >> 8;//[0,255], second largest component of this color
      let r$: number;
      let g$: number;
      let b$: number;
      if (h1 == 0) {
          r$ = c; g$ = x; b$ = 0;
      } else if (h1 == 1) {
          r$ = x; g$ = c; b$ = 0;
      } else if (h1 == 2) {
          r$ = 0; g$ = c; b$ = x;
      } else if (h1 == 3) {
          r$ = 0; g$ = x; b$ = c;
      } else if (h1 == 4) {
          r$ = x; g$ = 0; b$ = c;
      } else if (h1 == 5) {
          r$ = c; g$ = 0; b$ = x;
      }
      let m = Math.idiv((Math.idiv((l * 2 << 8), 100) - c), 2);
      let r = r$ + m;
      let g = g$ + m;
      let b = b$ + m;
      return packRGB(r, g, b);
  }

  export enum HueInterpolationDirection {
      Clockwise,
      CounterClockwise,
      Shortest
  }

  create()

}
