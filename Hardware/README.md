# Electrospinning Device based on Arduino Mega

## High voltage result and schematics

### Reult

<p> The result of the electrospinning process is accesible on link below. </p>
<a href="https://docs.google.com/spreadsheets/d/1FEkuDaI_2rEFF1M06S9z06Yd7cQ-smT4pYeCvJsIN_U/edit?usp=sharing">Value on testing process</a>

### Schematics

## Hardware Connection

<h3>Display LCD I2C 16x2</h3>
<table style="border: none;">
  <tr>
    <th>Arduino Pins</th>
    <th>LCD Pin</th>
  </tr>
  <tr>
    <td>20</td>
    <td>21</td>
  </tr>
  <tr>
    <td>SCL</td>
    <td>SDA</td>
  </tr>
</table>

<h3>Motor</h3>
<table style="border: none;">
  <tr>
    <th>Arduino Pins</th>
    <th>Driver TB6600</th>
    <th>Motor Pin</th>
  </tr>
  <tr>
    <td>48</td>
    <td>ENA</td>
    <td>Enable Motor</td>
  </tr>
  <tr>
    <td>46</td>
    <td>DIR</td>
    <td>Direction Motor</td>
  </tr>
  <tr>
    <td>44</td>
    <td>PUL</td>
    <td>Pulse Motor</td>
  </tr>

  <h3>Encoder</h3>
  <table style="border: none;">
  <tr>
    <th>Arduino Pins</th>
    <th>Driver TB6600</th>
    <th>Encoder Pin</th>
  </tr>
  <tr>
    <td>26</td>
    <td>CLK</td>
    <td>Encoder Clock</td>
  </tr>
  <tr>
    <td>24</td>
    <td>DT</td>
    <td>Encoder Data transfer</td>
  </tr>
  <tr>
    <td>22</td>
    <td>SW</td>
    <td>Encoder Switch select</td>
  </tr>
  </table>

<h3>ADC</h3>
<table style="border: none;">
  <tr>
    <th>Arduino Pins</th>
    <th>ADC Pin</th>
  </tr>
  <tr>
    <td>A9</td>
    <td>ADC</td>
  </tr>
</table>

<h3>PWM</h3>
<table style="border: none;">
  <tr>
    <th>Arduino Pins</th>
    <th>PWM Pin</th>
  </tr>
  <tr>
    <td>9</td>
    <td>PWM</td>
  </tr>
</table>

<h3>Relay</h3>
<table style="border: none;">
  <tr>
    <th>Arduino Pins</th>
    <th>Relay Pin</th>
  </tr>
  <tr>
    <td>15</td>
    <td>Relay</td>
  </tr>
</table>

