"""
CookingBot.py
Epicura Cooking Bot - Handles actual hardware control and cooking operations
"""

import time
import threading
from typing import Optional
from RecipeModel import CookingSegment, IngredientType, IngredientVector, TempProfile, StirringProfile, TemperatureProfileType


class CookingBot:
    """
    Handles the actual cooking operations and hardware control.
    Manages dispensers, temperature control, stirring mechanisms, and safety systems.
    """
    
    def __init__(self):
        """Initialize the cooking bot with hardware interfaces."""
        self.is_active = False
        self.current_segment = None
        
        # Hardware controller references (to be initialized with actual hardware)
        self.spi_controller = None  # TODO: Initialize SPI controller for Arduino/Pico communication
        self.temperature_controller = None  # TODO: Initialize temperature control system
        self.stirring_controller = None  # TODO: Initialize stirring motor controller
        self.safety_monitor = None  # TODO: Initialize safety monitoring system
        
        # State tracking
        self.current_temperature = 25  # Room temperature
        self.is_stirring = False
        self.stirring_rpm = 0
        
    def execute_segment(self, segment: CookingSegment, duration: float, stop_event: threading.Event):
        """
        Execute a cooking segment by controlling all subsystems.
        
        Args:
            segment: CookingSegment object to execute
            duration: Duration of the segment in seconds
            stop_event: Threading event to check for stop signals
        """
        self.is_active = True
        self.current_segment = segment
        
        print(f"  🤖 CookingBot: Executing segment {segment.segment_id}")
        
        # Step 1: Dispense ingredients
        self._dispense_ingredients(segment.ingredients, stop_event)
        
        # Step 2: Start temperature control
        self._start_temperature_control(segment.ISC, duration, stop_event)
        
        # Step 3: Start stirring control
        self._start_stirring_control(segment.stirring_profile, stop_event)
        
        # The scheduler will handle the timing, we just set everything up
        print(f"  ✅ CookingBot: Segment setup complete, cooking in progress...")
    
    def finish_segment(self, segment: CookingSegment):
        """
        Finish a cooking segment by stopping active operations.
        
        Args:
            segment: CookingSegment that is completing
        """
        print(f"  🤖 CookingBot: Finishing segment {segment.segment_id}")
        
        # Stop stirring
        self._stop_stirring()
        
        # Temperature control continues unless explicitly changed
        # (it will be updated by the next segment)
        
        self.is_active = False
        self.current_segment = None
        
        print(f"  ✅ CookingBot: Segment {segment.segment_id} finished")
    
    def _dispense_ingredients(self, ingredients: dict[IngredientType, IngredientVector], 
                             stop_event: threading.Event):
        """
        Dispense ingredients using appropriate dispensers.
        
        Args:
            ingredients: Dictionary mapping ingredient types to ingredient vectors
            stop_event: Threading event to check for stop signals
        """
        if not ingredients:
            print("  ├─ 📦 No ingredients to dispense")
            return
        
        print(f"  ├─ 📦 Dispensing {len(ingredients)} ingredient(s):")
        
        for ingredient_type, ingredient_vector in ingredients.items():
            if stop_event.is_set():
                print("  │  ⚠️  Dispensing interrupted by stop signal")
                break
            
            print(f"  │  ├─ {ingredient_type.value}")
            print(f"  │  │  └─ Index: {ingredient_vector.index}, "
                  f"Qty: {ingredient_vector.quantity} {ingredient_vector.unit}")
            
            # Dispatch to appropriate dispenser based on ingredient type
            if ingredient_type == IngredientType.ASD:
                self._dispense_spice(ingredient_vector)
            elif ingredient_type == IngredientType.SLD:
                self._dispense_standard_liquid(ingredient_vector)
            elif ingredient_type == IngredientType.VLD:
                self._dispense_viscous_liquid(ingredient_vector)
            elif ingredient_type == IngredientType.CID:
                self._dispense_coarse_ingredient(ingredient_vector)
            elif ingredient_type == IngredientType.LID:
                self._dispense_lumped_ingredient(ingredient_vector)
            else:
                print(f"  │  │  ⚠️  Unknown ingredient type: {ingredient_type}")
        
        print("  │")
    
    def _dispense_spice(self, ingredient: IngredientVector):
        """
        Dispense spice using Advanced Spice Dispenser (ASD).
        
        Args:
            ingredient: IngredientVector with cartridge index and quantity
        """
        # TODO: Send SPI command to Arduino controlling ASD
        # Command format: [DEVICE_ID, COMMAND, CARTRIDGE_INDEX, AMOUNT_HIGH, AMOUNT_LOW]
        # Example: spi_controller.send_command(DEVICE_ASD, CMD_DISPENSE, cartridge, amount)
        
        print(f"  │  │  🌶️  Activating spice cartridge {ingredient.index}")
        time.sleep(0.5)  # Simulate dispensing time
        print(f"  │  │  ✅ Dispensed {ingredient.quantity}{ingredient.unit} of spice")
    
    def _dispense_standard_liquid(self, ingredient: IngredientVector):
        """
        Dispense standard liquid using Standard Liquid Dispenser (SLD).
        
        Args:
            ingredient: IngredientVector with pump index and volume
        """
        # TODO: Send SPI command to Arduino/Pico controlling SLD
        # Calculate pump duration based on flow rate and desired volume
        # Monitor ultrasonic sensor for level verification
        
        print(f"  │  │  💧 Activating pump {ingredient.index}")
        time.sleep(0.5)  # Simulate dispensing time
        print(f"  │  │  ✅ Dispensed {ingredient.quantity}{ingredient.unit} of liquid")
    
    def _dispense_viscous_liquid(self, ingredient: IngredientVector):
        """
        Dispense viscous liquid using Viscous Liquid Dispenser (VLD).
        
        Args:
            ingredient: IngredientVector with pump index and volume
        """
        # TODO: Send SPI command to Arduino/Pico controlling VLD
        # Use peristaltic pump with slower flow rate for viscous liquids
        
        print(f"  │  │  🍯 Activating viscous pump {ingredient.index}")
        time.sleep(0.8)  # Simulate slower dispensing for viscous liquids
        print(f"  │  │  ✅ Dispensed {ingredient.quantity}{ingredient.unit} of viscous liquid")
    
    def _dispense_coarse_ingredient(self, ingredient: IngredientVector):
        """
        Dispense coarse ingredient using Coarse Ingredient Dispenser (CID).
        
        Args:
            ingredient: IngredientVector with hopper index and weight
        """
        # TODO: Send SPI command to Arduino controlling CID
        # Use servo/stepper to open gate, monitor load cell for weight
        
        print(f"  │  │  🥕 Opening hopper gate {ingredient.index}")
        time.sleep(0.6)  # Simulate dispensing time
        print(f"  │  │  ✅ Dispensed {ingredient.quantity}{ingredient.unit} of coarse ingredient")
    
    def _dispense_lumped_ingredient(self, ingredient: IngredientVector):
        """
        Dispense lumped ingredient using Lumped Ingredient Dispenser (LID).
        
        Args:
            ingredient: IngredientVector with servo index and weight
        """
        # TODO: Send SPI command to Arduino controlling LID
        # Use servo mechanism to push/drop chunked ingredients
        
        print(f"  │  │  🍖 Activating dispenser servo {ingredient.index}")
        time.sleep(0.7)  # Simulate dispensing time
        print(f"  │  │  ✅ Dispensed {ingredient.quantity}{ingredient.unit} of lumped ingredient")
    
    def _start_temperature_control(self, temp_profiles: list[TempProfile], 
                                   segment_duration: float, stop_event: threading.Event):
        """
        Start temperature control based on ISC (Induction Stove Control) profiles.
        
        Args:
            temp_profiles: List of temperature profiles to execute
            segment_duration: Total duration of the segment for profile timing
            stop_event: Threading event to check for stop signals
        """
        if not temp_profiles:
            print("  ├─ 🌡️  Temperature Control: None")
            return
        
        print(f"  ├─ 🌡️  Temperature Control ({len(temp_profiles)} profile(s)):")
        
        for idx, profile in enumerate(temp_profiles):
            print(f"  │  ├─ Profile {idx + 1}: {profile.profile_type.value}")
            print(f"  │  │  └─ {profile.temp_start}°C → {profile.temp_end}°C "
                  f"over {profile.duration}s")
            
            # Set up temperature profile based on type
            if profile.profile_type == TemperatureProfileType.LINEAR:
                self._setup_linear_temperature(profile, stop_event)
            elif profile.profile_type == TemperatureProfileType.STEPWISE:
                self._setup_stepwise_temperature(profile, stop_event)
            elif profile.profile_type == TemperatureProfileType.EXPONENTIAL:
                self._setup_exponential_temperature(profile, stop_event)
            elif profile.profile_type == TemperatureProfileType.QUADRATIC:
                self._setup_quadratic_temperature(profile, stop_event)
        
        print("  │")
    
    def _setup_linear_temperature(self, profile: TempProfile, stop_event: threading.Event):
        """
        Set up linear temperature ramping.
        
        Args:
            profile: Temperature profile with start/end temperatures and duration
            stop_event: Threading event to check for stop signals
        """
        # TODO: Implement linear temperature ramping
        # Calculate ramp rate: (temp_end - temp_start) / duration
        # Start thread to continuously update power level via capacitive touch interface
        # Monitor ToF sensor for pot presence
        # Use PID control for temperature regulation
        
        ramp_rate = (profile.temp_end - profile.temp_start) / profile.duration
        print(f"  │  │  📈 Linear ramp: {ramp_rate:.2f}°C/s")
        self.current_temperature = profile.temp_start
    
    def _setup_stepwise_temperature(self, profile: TempProfile, stop_event: threading.Event):
        """
        Set up stepwise temperature change (immediate jump to target).
        
        Args:
            profile: Temperature profile with target temperature
            stop_event: Threading event to check for stop signals
        """
        # TODO: Implement immediate temperature change
        # Set power level to achieve target temperature as quickly as possible
        
        print(f"  │  │  📊 Stepwise change to {profile.temp_end}°C")
        self.current_temperature = profile.temp_end
    
    def _setup_exponential_temperature(self, profile: TempProfile, stop_event: threading.Event):
        """
        Set up exponential temperature curve.
        
        Args:
            profile: Temperature profile with exponential curve parameters
            stop_event: Threading event to check for stop signals
        """
        # TODO: Implement exponential temperature curve
        # Use exponential function to calculate temperature at each time step
        
        print(f"  │  │  📉 Exponential curve")
        self.current_temperature = profile.temp_start
    
    def _setup_quadratic_temperature(self, profile: TempProfile, stop_event: threading.Event):
        """
        Set up quadratic temperature curve.
        
        Args:
            profile: Temperature profile with quadratic curve parameters
            stop_event: Threading event to check for stop signals
        """
        # TODO: Implement quadratic temperature curve
        # Use quadratic function to calculate temperature at each time step
        
        print(f"  │  │  📐 Quadratic curve")
        self.current_temperature = profile.temp_start
    
    def _start_stirring_control(self, stirring_profiles: list[StirringProfile], 
                               stop_event: threading.Event):
        """
        Start stirring mechanism based on stirring profiles.
        
        Args:
            stirring_profiles: List of stirring profiles to execute
            stop_event: Threading event to check for stop signals
        """
        if not stirring_profiles:
            print("  ├─ 🥄 Stirring: None")
            return
        
        print(f"  ├─ 🥄 Stirring Control ({len(stirring_profiles)} profile(s)):")
        
        for idx, profile in enumerate(stirring_profiles):
            print(f"  │  ├─ Profile {idx + 1}: {profile.profile_type.value}")
            print(f"  │  │  └─ {profile.rpm} RPM for {profile.duration}s")
            
            # Start stirring with the specified profile
            self._activate_stirring(profile, stop_event)
        
        print("  │")
    
    def _activate_stirring(self, profile: StirringProfile, stop_event: threading.Event):
        """
        Activate stirring motor with specified profile.
        
        Args:
            profile: Stirring profile with RPM and pattern
            stop_event: Threading event to check for stop signals
        """
        # TODO: Send commands to stirring motor controller
        # Determine direction from profile type (CW/CCW/Alternating)
        # Set motor PWM for target RPM
        # For intermittent profiles, implement on/off timing
        # For variable profiles, implement RPM ramping
        
        profile_type = profile.profile_type.value
        
        if "Clockwise" in profile_type:
            direction = "CW"
        elif "CounterClockwise" in profile_type:
            direction = "CCW"
        elif "Alternating" in profile_type:
            direction = "ALT"
        else:
            direction = "UNKNOWN"
        
        if "Constant" in profile_type:
            pattern = "CONSTANT"
        elif "Variable" in profile_type:
            pattern = "VARIABLE"
        elif "Intermittent" in profile_type:
            pattern = "INTERMITTENT"
        else:
            pattern = "UNKNOWN"
        
        print(f"  │  │  🔄 Stirring: {direction} {pattern} @ {profile.rpm} RPM")
        
        self.is_stirring = True
        self.stirring_rpm = profile.rpm
    
    def _stop_stirring(self):
        """Stop the stirring mechanism."""
        if self.is_stirring:
            print("  └─ 🛑 Stopping stirring mechanism")
            # TODO: Send stop command to stirring motor controller
            # motor_controller.set_rpm(0)
            
            self.is_stirring = False
            self.stirring_rpm = 0
        else:
            print("  └─ ℹ️  Stirring already stopped")
    
    def emergency_stop(self):
        """
        Emergency stop all cooking subsystems immediately.
        Called when user stops cooking or in case of emergency.
        """
        print("  🚨 [EMERGENCY STOP] Stopping all subsystems...")
        
        # Stop stirring
        if self.is_stirring:
            self._stop_stirring()
        
        # Turn off induction cooker
        print("  │  🔴 Turning off induction cooker")
        # TODO: Send emergency stop to induction controller
        # induction_controller.set_power(0)
        self.current_temperature = 25  # Will cool down naturally
        
        # Stop all dispensers
        print("  │  🔴 Stopping all dispensers")
        # TODO: Send stop commands to all dispenser controllers
        # spi_controller.emergency_stop_all()
        
        # Close all valves
        print("  │  🔴 Closing all valves")
        # TODO: Close all liquid dispenser valves
        
        # Activate safety alerts if needed
        print("  │  ⚠️  Checking safety systems")
        # TODO: Check LPG leak detector, temperature sensors, etc.
        
        self.is_active = False
        self.current_segment = None
        
        print("  ✅ [EMERGENCY STOP COMPLETE] All subsystems stopped")
    
    def get_hardware_status(self) -> dict:
        """
        Get current hardware status.
        
        Returns:
            Dictionary containing hardware status information
        """
        return {
            "is_active": self.is_active,
            "current_segment": self.current_segment.segment_id if self.current_segment else None,
            "current_temperature": self.current_temperature,
            "is_stirring": self.is_stirring,
            "stirring_rpm": self.stirring_rpm,
            "spi_connected": self.spi_controller is not None,
            "temperature_controller_connected": self.temperature_controller is not None,
            "stirring_controller_connected": self.stirring_controller is not None,
        }
    
    def initialize_hardware(self):
        """
        Initialize all hardware interfaces and controllers.
        Should be called before starting to cook.
        """
        print("🔧 Initializing hardware interfaces...")
        
        # TODO: Initialize SPI controller
        # self.spi_controller = SPIController(spi_bus=0, cs_pins=[...])
        print("  ├─ SPI Controller: TODO")
        
        # TODO: Initialize temperature controller
        # self.temperature_controller = InductionController(interface=capacitive_touch)
        print("  ├─ Temperature Controller: TODO")
        
        # TODO: Initialize stirring controller
        # self.stirring_controller = StirringMotorController(pwm_pin=..., encoder_pin=...)
        print("  ├─ Stirring Controller: TODO")
        
        # TODO: Initialize safety monitoring
        # self.safety_monitor = SafetyMonitor(lpg_sensor_pin=..., temp_sensors=[...])
        print("  └─ Safety Monitor: TODO")
        
        print("✅ Hardware initialization complete (simulated)")
