<h2>Contents</h2>

<ul>
  <li><a href="#features">Features</a></li>
  <li><a href="#hardware-setup">Hardware Setup</a></li>
  <li><a href="#software-board-code">Software Board Code</a></li>
  <li><a href="#usage">Usage</a></li>
  <li><a href="#contributing">Contributing</a></li>
  <li><a href="#license">License</a></li>
</ul>

<h1>Electrospinning Device based on ATMEGA2560</h1>

<p>This repository contains the source code, hardware schematics, and documentation for a device designed to facilitate the electrospinning process using the ATMEGA2560 microcontroller (Arduino Mega).</p>

<h2 id="features">Features</h2>

<p>The device provides the following features:</p>
<ul>
  <li>Control and monitoring with BME280 sensor of temperature, pressure, humidity. And adjustable flow rate.</li>
  <li>FreeRTOS based task scheduler for easier and more efficient task management.</li>
  <li>High voltage source for electrospinning process.</li>
  <li>LCD display for monitoring.</li>
  <li>Motor for controlling the flow of the syringe. using driver TB6600.</li>
  <li>Encoder for user interaction.</li>
</ul>

<h2 id="hardware-setup">Hardware Setup</h2>

<table>
  <tr>
    <th>
      <p>The hardware setup consists of the following components:</p>
    </th>
  <tr>
    <td>
      <ul>
        <li><b>The high voltage power supply</b> generates the high voltage needed for the electrospinning process. It provides a stable and adjustable voltage source to create the electrostatic field required to draw the polymer solution into nanofibers.</li>
        <li><b>Varistor and Transformer</b> convert the high voltage to the voltage required for electrospinning.</li>
        <li><b>Multiplier/Rectifier</b> converts the voltage to the voltage required for the motor.</li>
      </ul>
    </td>
  </tr>
  <tr>
    <td>
      <h3>Comand Board</h3>
        <ul>
        <li><b>MCU</b> ATMEGA2560 (Mega 2560)</li>
        <li><b>Encoder</b> For seting the flow rate, starting and stopping the motor, and opening screen with sensor values.</li>
        <li><b>LCD Display</b> Show sensor values.</li>
        <li><b>TB6600</b> Stepper motor driver.</li>
        <li><b>Motor</b> Stepper motor for controlling the flow of the syringe.</li>
        <li><b>BME280</b> Sensor for monitoring temperature, pressure, and humidity.</li>
        <li><b>Power Supply</b> MCU power supply.</li>
      </ul>
    </td>
  </tr>
</table>
<ul>
  <li>See more about <a href="/Hardware/README.md">hardware Setup</a></li>
</ul>


<h2 id="software-board-code">Software Board Code</h2>
<ul>
  <li>See more about <a href="/Software/Source code">software board code.</a></li>
  <!-- Builded file is ... -->
  <li>Get hex, elf files: <a href="/Software/Source code/build/">software board code.</a></li>
</ul>

<h2>Usage</h2>

<p>To use the electrospinning device:</p>
<ol>
  <li>Connect the high voltage source to 220V.</li>
  <li>Connect the Arduino Mega to the components and upload the provided source code.</li>
  <li>Use the LCD screen with encoder to control and monitor the electrospinning process.</li>
  <li>Adjust the settings as needed and start the electrospinning process.</li>
  <li>Monitor the output and make adjustments as necessary.</li>
</ol>

<h2>Contributing</h2>
<p>Contributions to this project are welcome. Please fork the repository, make your changes, and submit a pull request.</p>

<h2>License</h2>
<p>This project is licensed under the <a href="LICENSE">MIT License</a>.</p>


