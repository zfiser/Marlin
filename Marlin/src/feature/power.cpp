/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

/**
 * power.cpp - power control
 */

#include "../inc/MarlinConfig.h"

#if ENABLED(AUTO_POWER_CONTROL)

#include "power.h"
#include "../module/temperature.h"
#include "../module/stepper/indirection.h"
#include "../MarlinCore.h"

#if BOTH(USE_CONTROLLER_FAN, AUTO_POWER_CONTROLLERFAN)
  #include "controllerfan.h"
#endif

Power powerManager;

millis_t Power::lastPowerOn;
millis_t Power::bootUpTime = millis();

bool Power::is_power_needed() {
  /*millis_t end = bootUpTime+(5*60*1000UL);
  if (!ELAPSED(millis(), end )) {
    DEBUG_SERIAL_ECHOPGM("is_power_needed(): boottime + 5 mins, left: ");
    DEBUG_SERIAL_ECHO(end-millis());
    DEBUG_SERIAL_ECHOPGM(", end: ");
    DEBUG_SERIAL_ECHO(end);
    DEBUG_SERIAL_EOL(); 
    return true;
  }*/
  #if ENABLED(AUTO_POWER_FANS)
    FANS_LOOP(i) if (thermalManager.fan_speed[i]) {
      DEBUG_SERIAL_ECHO_START();
      DEBUG_SERIAL_ECHOPGM("is_power_needed(): AUTO_POWER_FANS");
      DEBUG_SERIAL_ECHOPGM(", powersupply_on: ");
      DEBUG_SERIAL_ECHO(powersupply_on);
      DEBUG_SERIAL_EOL(); 
      return true;
    }
  #endif

  #if ENABLED(AUTO_POWER_E_FANS)
    HOTEND_LOOP() if (thermalManager.autofan_speed[e]) {
      DEBUG_SERIAL_ECHO_START();
      DEBUG_SERIAL_ECHOPGM("is_power_needed(): AUTO_POWER_E_FANS");
      DEBUG_SERIAL_EOL(); 
      return true;
    }
  #endif

  #if BOTH(USE_CONTROLLER_FAN, AUTO_POWER_CONTROLLERFAN)
    if (controllerFan.state()) {
      DEBUG_SERIAL_ECHO_START();
      DEBUG_SERIAL_ECHOPGM("is_power_needed(): AUTO_POWER_E_FANS");
      DEBUG_SERIAL_EOL(); 
      return true;
    }
  #endif

  if (TERN0(AUTO_POWER_CHAMBER_FAN, thermalManager.chamberfan_speed)) {
    DEBUG_SERIAL_ECHO_START();
    DEBUG_SERIAL_ECHOPGM("is_power_needed(): AUTO_POWER_CHAMBER_FAN");
    DEBUG_SERIAL_EOL(); 
    return true;
  }

  // If any of the drivers or the bed are enabled...
  bool X= X_ENABLE_READ() == X_ENABLE_ON;
  bool Y= Y_ENABLE_READ() == Y_ENABLE_ON;
  bool Z= Z_ENABLE_READ() == Z_ENABLE_ON;
  bool E1= E1_ENABLE_READ() == E_ENABLE_ON;
  bool E2= E2_ENABLE_READ() == E_ENABLE_ON;
  if (X_ENABLE_READ() == X_ENABLE_ON || Y_ENABLE_READ() == Y_ENABLE_ON || Z_ENABLE_READ() == Z_ENABLE_ON
    #if HAS_X2_ENABLE
      || X2_ENABLE_READ() == X_ENABLE_ON
    #endif
    #if HAS_Y2_ENABLE
      || Y2_ENABLE_READ() == Y_ENABLE_ON
    #endif
    #if HAS_Z2_ENABLE
      || Z2_ENABLE_READ() == Z_ENABLE_ON
    #endif
    #if E_STEPPERS
      #define _OR_ENABLED_E(N) || E##N##_ENABLE_READ() == E_ENABLE_ON
      REPEAT(E_STEPPERS, _OR_ENABLED_E)
    #endif
  ) {
    DEBUG_SERIAL_ECHO_START();
    DEBUG_SERIAL_ECHOPGM("is_power_needed(): DRIVER ENABLED:");
    if (X)  DEBUG_SERIAL_ECHOPGM(" X ");
    if (Y)  DEBUG_SERIAL_ECHOPGM(" Y ");
    if (Z)  DEBUG_SERIAL_ECHOPGM(" Z");
    if (E1) DEBUG_SERIAL_ECHOPGM(" E1 ");
    if (E2) DEBUG_SERIAL_ECHOPGM(" E2 ");
    DEBUG_SERIAL_ECHOPGM(", powersupply_on: ");
    DEBUG_SERIAL_ECHO(powersupply_on);
    DEBUG_SERIAL_EOL(); 
    return true;
  }

  HOTEND_LOOP() if (thermalManager.degTargetHotend(e) > 0 || thermalManager.temp_hotend[e].soft_pwm_amount > 0) {
    DEBUG_SERIAL_ECHO_START();
    DEBUG_SERIAL_ECHOPGM("is_power_needed(): degTargetHotend()");
    DEBUG_SERIAL_EOL(); 
    return true;
  }
  if (TERN0(HAS_HEATED_BED, thermalManager.degTargetBed() > 0 || thermalManager.temp_bed.soft_pwm_amount > 0)) {
    DEBUG_SERIAL_ECHO_START();
    DEBUG_SERIAL_ECHOPGM("is_power_needed(): degTargetBed()");
    DEBUG_SERIAL_EOL(); 
    return true;
  }

  #if HAS_HOTEND && AUTO_POWER_E_TEMP
    HOTEND_LOOP() if (thermalManager.degHotend(e) >= AUTO_POWER_E_TEMP) {
      DEBUG_SERIAL_ECHO_START();
      DEBUG_SERIAL_ECHOPGM("is_power_needed(): AUTO_POWER_E_TEMP");
      DEBUG_SERIAL_EOL(); 
      return true;
    }
  #endif

  #if HAS_HEATED_CHAMBER && AUTO_POWER_CHAMBER_TEMP
    if (thermalManager.degChamber() >= AUTO_POWER_CHAMBER_TEMP) {
      DEBUG_SERIAL_ECHO_START();
      DEBUG_SERIAL_ECHOPGM("is_power_needed(): AUTO_POWER_CHAMBER_TEMP");
      DEBUG_SERIAL_EOL(); 
      return true;
    }
  #endif

  DEBUG_SERIAL_ECHO_START();
  DEBUG_SERIAL_ECHOPGM("is_power_needed(): NOT NEEDED");
  DEBUG_SERIAL_ECHOPGM(", powersupply_on: ");
  DEBUG_SERIAL_ECHO(powersupply_on);
  DEBUG_SERIAL_EOL(); 
  return false;
}

void Power::check() {
  static millis_t nextPowerCheck = 0;
  millis_t ms = millis();

  if (ELAPSED(ms, nextPowerCheck)) {
    /*DEBUG_SERIAL_ECHO_START();
    DEBUG_SERIAL_ECHOPGM("Power::CHECK(): lastPowerOn:");
    DEBUG_SERIAL_ECHO(lastPowerOn);
    DEBUG_SERIAL_ECHOPGM(", ms: ");
    DEBUG_SERIAL_ECHO(ms);
    DEBUG_SERIAL_ECHOPGM(", diff: ");
    DEBUG_SERIAL_ECHO(ms-lastPowerOn);
    DEBUG_SERIAL_EOL();
    */
    nextPowerCheck = ms + 2500UL;
    if (is_power_needed())
      power_on("Power::check() power_on is_power_needed");
    else if (!lastPowerOn || ELAPSED(ms, lastPowerOn + SEC_TO_MS(POWER_TIMEOUT)))
      power_off("Power::check() power_off is_power_needed");
  }
}

void Power::power_on(const char *from) {
  lastPowerOn = millis();

  DEBUG_SERIAL_ECHO_START();
  DEBUG_SERIAL_ECHOPGM("power_on(");
  DEBUG_SERIAL_ECHO(from);
  DEBUG_SERIAL_ECHOPGM("), enter: powersupply_on: ");
  DEBUG_SERIAL_ECHO(powersupply_on);
  if (!powersupply_on) {
    PSU_PIN_ON();
    safe_delay(PSU_POWERUP_DELAY);
    restore_stepper_drivers();
    TERN_(HAS_TRINAMIC_CONFIG, safe_delay(PSU_POWERUP_DELAY));
  }
  DEBUG_SERIAL_ECHOPGM("), leave: powersupply_on: ");
  DEBUG_SERIAL_ECHO(powersupply_on);
  DEBUG_SERIAL_EOL();
}


void Power::power_off(const char *from) {
  DEBUG_SERIAL_ECHO_START();
  DEBUG_SERIAL_ECHOPGM("power_off(");
  DEBUG_SERIAL_ECHO(from);
  DEBUG_SERIAL_ECHOPGM(") powersupply_on: ");
  DEBUG_SERIAL_ECHO(powersupply_on);
  DEBUG_SERIAL_EOL();
  if (powersupply_on) {
    PSU_PIN_OFF();
  }
  #if PSU_POWEROFF_DELAY>0
    safe_delay(PSU_POWEROFF_DELAY);
  #endif
}

#endif // AUTO_POWER_CONTROL
