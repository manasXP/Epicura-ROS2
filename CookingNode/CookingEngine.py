"""
CookingEngine.py
Epicura Cooking Engine - Processes recipes by scheduling cooking segments
"""

import time
import threading
from typing import Callable, Optional
from datetime import datetime, timedelta
from RecipeModel import Recipe, CookingSegment, IngredientType, IngredientVector, TempProfile, StirringProfile


class CookingEngine:
    """
    Main cooking engine that processes recipes by scheduling and executing
    cooking segments with precise timing control based on segment_schedule.
    """
    
    def __init__(self):
        self.is_cooking = False
        self.current_segment = None
        self.current_segment_index = None
        self.current_recipe = None
        self.segment_timer = None
        self.cooking_thread = None
        self.stop_event = threading.Event()
        self.recipe_start_time = None
        
    def cook_recipe(self, recipe: Recipe, 
                    on_segment_start: Optional[Callable[[CookingSegment, int], None]] = None,
                    on_segment_complete: Optional[Callable[[CookingSegment, int], None]] = None,
                    on_recipe_complete: Optional[Callable[[Recipe], None]] = None,
                    on_error: Optional[Callable[[Exception], None]] = None):
        """
        Cook a recipe by processing each cooking segment according to segment_schedule.
        
        Args:
            recipe: Recipe object containing cooking segments to process
            on_segment_start: Callback function called when a segment starts (segment, segment_index)
            on_segment_complete: Callback function called when a segment completes (segment, segment_index)
            on_recipe_complete: Callback function called when entire recipe is complete
            on_error: Callback function called if an error occurs during cooking
        """
        if self.is_cooking:
            raise RuntimeError("Engine is already cooking a recipe. Stop current recipe first.")
        
        self.current_recipe = recipe
        self.is_cooking = True
        self.stop_event.clear()
        
        # Start cooking in a separate thread to avoid blocking
        self.cooking_thread = threading.Thread(
            target=self._cook_recipe_thread,
            args=(recipe, on_segment_start, on_segment_complete, on_recipe_complete, on_error),
            daemon=True
        )
        self.cooking_thread.start()
        
    def _cook_recipe_thread(self, recipe: Recipe,
                           on_segment_start: Optional[Callable],
                           on_segment_complete: Optional[Callable],
                           on_recipe_complete: Optional[Callable],
                           on_error: Optional[Callable]):
        """Internal method that runs in a separate thread to process recipe segments."""
        try:
            # Get the segment schedule (cumulative times)
            segment_schedule = recipe.segment_schedule
            
            print(f"\n{'='*60}")
            print(f"Starting Recipe: {recipe.name}")
            print(f"Total Cooking Time: {recipe.cooking_time} seconds ({recipe.cooking_time // 60} min {recipe.cooking_time % 60} sec)")
            print(f"Number of Segments: {recipe.segment_count}")
            print(f"Segment Schedule: {segment_schedule}")
            print(f"{'='*60}\n")
            
            # Record recipe start time
            self.recipe_start_time = time.time()
            
            # Process each segment according to schedule
            for idx, segment in enumerate(recipe.cooking_segments):
                if self.stop_event.is_set():
                    print("\n[STOPPED] Cooking stopped by user.")
                    break
                
                self.current_segment = segment
                self.current_segment_index = idx
                
                # Calculate when this segment should start and end
                segment_start_time = 0 if idx == 0 else segment_schedule[idx - 1]
                segment_end_time = segment_schedule[idx]
                
                # Calculate elapsed time since recipe start
                elapsed_time = time.time() - self.recipe_start_time
                
                # Wait if we're ahead of schedule
                if elapsed_time < segment_start_time:
                    wait_time = segment_start_time - elapsed_time
                    print(f"[TIMING] Waiting {wait_time:.1f}s to maintain schedule...")
                    time.sleep(wait_time)
                    elapsed_time = time.time() - self.recipe_start_time
                
                # Check if we're behind schedule
                if elapsed_time > segment_start_time + 1.0:  # 1 second tolerance
                    delay = elapsed_time - segment_start_time
                    print(f"[WARNING] Segment {idx + 1} starting {delay:.1f}s behind schedule")
                
                # Call segment start callback
                if on_segment_start:
                    on_segment_start(segment, idx)
                
                # Process the segment with scheduled timing
                self._process_segment_scheduled(segment, idx, segment_start_time, segment_end_time)
                
                # Call segment complete callback
                if on_segment_complete:
                    on_segment_complete(segment, idx)
            
            # Recipe completed successfully
            if not self.stop_event.is_set():
                total_time = time.time() - self.recipe_start_time
                print(f"\n{'='*60}")
                print(f"Recipe Complete: {recipe.name}")
                print(f"Actual Total Time: {total_time:.1f}s")
                print(f"Scheduled Time: {recipe.cooking_time}s")
                print(f"Time Variance: {total_time - recipe.cooking_time:.1f}s")
                print(f"{'='*60}\n")
                
                if on_recipe_complete:
                    on_recipe_complete(recipe)
                    
        except Exception as e:
            print(f"\n[ERROR] Cooking error: {e}")
            if on_error:
                on_error(e)
        finally:
            self.is_cooking = False
            self.current_segment = None
            self.current_segment_index = None
            self.current_recipe = None
            self.recipe_start_time = None
    
    def _process_segment_scheduled(self, segment: CookingSegment, segment_index: int,
                                   scheduled_start: float, scheduled_end: float):
        """
        Process a single cooking segment with precise scheduled timing.
        
        Args:
            segment: CookingSegment object to process
            segment_index: Index of the segment in the recipe
            scheduled_start: Scheduled start time (seconds from recipe start)
            scheduled_end: Scheduled end time (seconds from recipe start)
        """
        segment_duration = scheduled_end - scheduled_start
        
        # Calculate absolute times for display
        start_datetime = datetime.now()
        end_datetime = start_datetime + timedelta(seconds=segment_duration)
        
        print(f"\n{'─'*60}")
        print(f"Segment {segment_index + 1} of {len(self.current_recipe.cooking_segments)} (ID: {segment.segment_id})")
        print(f"Duration: {segment_duration}s ({segment_duration // 60}m {segment_duration % 60:.0f}s)")
        print(f"Scheduled: {scheduled_start}s → {scheduled_end}s (from recipe start)")
        print(f"Clock Time: {start_datetime.strftime('%H:%M:%S')} → {end_datetime.strftime('%H:%M:%S')}")
        print(f"{'─'*60}")
        
        # Dispense ingredients at the start of the segment
        self._dispense_ingredients(segment.ingredients)
        
        # Start temperature control
        self._start_temperature_control(segment.ISC, segment_duration)
        
        # Start stirring control
        self._start_stirring_control(segment.stirring_profile)
        
        # Wait for the segment to complete according to schedule
        segment_actual_start = time.time()
        target_segment_end = self.recipe_start_time + scheduled_end
        
        # Monitor segment execution
        while time.time() < target_segment_end:
            if self.stop_event.is_set():
                break
            
            # Calculate progress
            elapsed = time.time() - segment_actual_start
            progress = (elapsed / segment_duration) * 100
            remaining = target_segment_end - time.time()
            
            # Print progress every 10 seconds (optional - can be removed for cleaner output)
            if int(elapsed) % 10 == 0 and elapsed > 0:
                print(f"  [Progress] {progress:.1f}% complete, {remaining:.0f}s remaining")
                time.sleep(1)  # Prevent multiple prints in the same second
            
            time.sleep(0.1)  # Check every 100ms for responsiveness
        
        # Stop stirring at end of segment
        self._stop_stirring()
        
        actual_duration = time.time() - segment_actual_start
        variance = actual_duration - segment_duration
        
        print(f"\n  Segment {segment_index + 1} Complete")
        print(f"  Actual Duration: {actual_duration:.1f}s (variance: {variance:+.1f}s)")
        print(f"  End Time: {datetime.now().strftime('%H:%M:%S')}\n")
    
    def _dispense_ingredients(self, ingredients: dict[IngredientType, IngredientVector]):
        """
        Dispense ingredients using appropriate dispensers.
        
        Args:
            ingredients: Dictionary mapping ingredient types to ingredient vectors
        """
        if not ingredients:
            print("  ├─ No ingredients to dispense")
            return
        
        print(f"  ├─ Dispensing {len(ingredients)} ingredient(s):")
        for ingredient_type, ingredient_vector in ingredients.items():
            print(f"  │  ├─ {ingredient_type.value}")
            print(f"  │  │  └─ Index: {ingredient_vector.index}, "
                  f"Qty: {ingredient_vector.quantity} {ingredient_vector.unit}")
            
            # TODO: Send SPI commands to appropriate dispenser based on ingredient_type
            # Example SPI command structure:
            # - ASD: spi.send_to_spice_dispenser(cartridge=index, amount=quantity)
            # - SLD: spi.send_to_liquid_dispenser(pump=index, volume=quantity)
            # - CID: spi.send_to_coarse_dispenser(hopper=index, weight=quantity)
            # - LID: spi.send_to_lumped_dispenser(servo=index, weight=quantity)
            # - VLD: spi.send_to_viscous_dispenser(pump=index, volume=quantity)
            
            # Simulate dispensing with a small delay
            time.sleep(0.5)
        
        print("  │")
    
    def _start_temperature_control(self, temp_profiles: list[TempProfile], segment_duration: float):
        """
        Start temperature control based on ISC (Induction Stove Control) profiles.
        
        Args:
            temp_profiles: List of temperature profiles to execute
            segment_duration: Total duration of the segment for profile timing
        """
        if not temp_profiles:
            print("  ├─ Temperature Control: None")
            return
        
        print(f"  ├─ Temperature Control ({len(temp_profiles)} profile(s)):")
        for idx, profile in enumerate(temp_profiles):
            print(f"  │  ├─ Profile {idx + 1}: {profile.profile_type.value}")
            print(f"  │  │  └─ {profile.temp_start}°C → {profile.temp_end}°C "
                  f"over {profile.duration}s")
        
        print("  │")
        
        # TODO: Send commands to induction cooker control via optocouplers/capacitive touch
        # Implementation would involve:
        # 1. Calculate temperature setpoints based on profile_type (LINEAR/STEPWISE/etc.)
        # 2. Send initial power level via capacitive touch interface
        # 3. Start monitoring loop with ToF sensor for pot presence
        # 4. Implement PID control loop for temperature regulation
        # 5. Update power levels throughout the profile duration
        
        # Example pseudo-code:
        # for profile in temp_profiles:
        #     if profile.profile_type == TemperatureProfileType.LINEAR:
        #         temp_ramp_rate = (profile.temp_end - profile.temp_start) / profile.duration
        #         # Start thread to continuously adjust temperature
        #     elif profile.profile_type == TemperatureProfileType.STEPWISE:
        #         # Set immediate target temperature
        #         induction_controller.set_temp(profile.temp_end)
    
    def _start_stirring_control(self, stirring_profiles: list[StirringProfile]):
        """
        Start stirring mechanism based on stirring profiles.
        
        Args:
            stirring_profiles: List of stirring profiles to execute
        """
        if not stirring_profiles:
            print("  ├─ Stirring: None")
            return
        
        print(f"  ├─ Stirring Control ({len(stirring_profiles)} profile(s)):")
        for idx, profile in enumerate(stirring_profiles):
            print(f"  │  ├─ Profile {idx + 1}: {profile.profile_type.value}")
            print(f"  │  │  └─ {profile.rpm} RPM for {profile.duration}s")
        
        print("  │")
        
        # TODO: Send commands to stirring motor controller
        # Implementation would involve:
        # 1. Determine motor direction from profile_type (CW/CCW/Alternating)
        # 2. Set motor PWM for target RPM
        # 3. For intermittent profiles, implement on/off timing
        # 4. For variable profiles, implement RPM ramping
        # 5. Monitor motor encoder for actual RPM feedback
        
        # Example pseudo-code:
        # for profile in stirring_profiles:
        #     direction = parse_direction(profile.profile_type)
        #     if "CONSTANT" in profile.profile_type.value:
        #         motor_controller.set_constant_rpm(profile.rpm, direction)
        #     elif "VARIABLE" in profile.profile_type.value:
        #         motor_controller.set_variable_rpm(profile.rpm, direction, profile.duration)
        #     elif "INTERMITTENT" in profile.profile_type.value:
        #         motor_controller.set_intermittent(profile.rpm, direction, on_time=5, off_time=5)
    
    def _stop_stirring(self):
        """Stop the stirring mechanism."""
        print("  └─ Stopping stirring mechanism")
        # TODO: Send stop command to stirring motor controller
        # motor_controller.stop()
    
    def stop_cooking(self):
        """Stop the current cooking process gracefully."""
        if self.is_cooking:
            print("\n[STOPPING] Stopping cooking process...")
            self.stop_event.set()
            
            # Emergency stop all subsystems
            self._emergency_stop_all_subsystems()
            
            if self.cooking_thread:
                self.cooking_thread.join(timeout=5.0)
            
            self.is_cooking = False
            self.current_segment = None
            self.current_segment_index = None
            self.current_recipe = None
            self.recipe_start_time = None
            print("[STOPPED] Cooking process stopped successfully")
    
    def _emergency_stop_all_subsystems(self):
        """Emergency stop all cooking subsystems."""
        print("  [EMERGENCY STOP] Stopping all subsystems...")
        # TODO: Implement emergency stop for all subsystems
        # - Stop all dispensers
        # - Turn off induction cooker
        # - Stop stirring motor
        # - Close all valves
        # - Activate safety alerts if needed
    
    def get_status(self) -> dict:
        """
        Get current cooking status with timing information.
        
        Returns:
            Dictionary containing current cooking status
        """
        if not self.is_cooking or not self.current_recipe:
            return {
                "is_cooking": False,
                "current_recipe": None,
                "current_segment": None,
                "recipe_id": None,
                "elapsed_time": 0,
                "remaining_time": 0,
                "progress_percent": 0
            }
        
        elapsed = time.time() - self.recipe_start_time if self.recipe_start_time else 0
        total_time = self.current_recipe.cooking_time
        remaining = max(0, total_time - elapsed)
        progress = (elapsed / total_time * 100) if total_time > 0 else 0
        
        return {
            "is_cooking": self.is_cooking,
            "current_recipe": self.current_recipe.name,
            "current_segment": self.current_segment.segment_id if self.current_segment else None,
            "current_segment_index": self.current_segment_index,
            "recipe_id": self.current_recipe.id,
            "elapsed_time": elapsed,
            "remaining_time": remaining,
            "progress_percent": min(100, progress),
            "segment_schedule": self.current_recipe.segment_schedule
        }
    
    def get_remaining_time(self) -> float:
        """Get remaining time in seconds for current recipe."""
        if not self.is_cooking or not self.recipe_start_time:
            return 0
        
        elapsed = time.time() - self.recipe_start_time
        return max(0, self.current_recipe.cooking_time - elapsed)


# Example usage and testing
if __name__ == "__main__":
    from RecipeModel import *
    
    # Create a sample recipe for testing
    def create_test_recipe() -> Recipe:
        """Create a test recipe with multiple segments for schedule testing."""
        
        # Segment 1: Heat oil and add spices (30 seconds)
        segment1 = CookingSegment(
            segment_id=1,
            duration=30,
            ingredients={
                IngredientType.SLD: IngredientVector(index=0, quantity=50, unit="ml"),  # Oil
                IngredientType.ASD: IngredientVector(index=1, quantity=5, unit="g")     # Cumin
            },
            ISC=[
                TempProfile(temp_start=25, temp_end=180, duration=30, 
                           profile_type=TemperatureProfileType.LINEAR)
            ],
            stirring_profile=[
                StirringProfile(duration=30, rpm=200, 
                              profile_type=StirringProfileType.CLOCKWISE_CONSTANT_RPM)
            ]
        )
        
        # Segment 2: Add vegetables and cook (45 seconds)
        segment2 = CookingSegment(
            segment_id=2,
            duration=45,
            ingredients={
                IngredientType.CID: IngredientVector(index=0, quantity=200, unit="g"),  # Onions
                IngredientType.LID: IngredientVector(index=1, quantity=150, unit="g")   # Tomatoes
            },
            ISC=[
                TempProfile(temp_start=180, temp_end=180, duration=45,
                           profile_type=TemperatureProfileType.STEPWISE)
            ],
            stirring_profile=[
                StirringProfile(duration=45, rpm=150,
                              profile_type=StirringProfileType.CLOCKWISE_INTERMITTENT_RPM)
            ]
        )
        
        # Segment 3: Add liquid and simmer (60 seconds)
        segment3 = CookingSegment(
            segment_id=3,
            duration=60,
            ingredients={
                IngredientType.SLD: IngredientVector(index=2, quantity=100, unit="ml")  # Water
            },
            ISC=[
                TempProfile(temp_start=180, temp_end=95, duration=60,
                           profile_type=TemperatureProfileType.LINEAR)
            ],
            stirring_profile=[
                StirringProfile(duration=60, rpm=100,
                              profile_type=StirringProfileType.ALTERNATING_CONSTANT_RPM)
            ]
        )
        
        recipe = Recipe(
            id="test_recipe_001",
            name="Test Curry Recipe",
            cooking_segments=[segment1, segment2, segment3]
        )
        
        return recipe
    
    # Callback functions
    def on_segment_start(segment: CookingSegment, idx: int):
        print(f"\n[CALLBACK] ✓ Segment {idx + 1} started (ID: {segment.segment_id})")
    
    def on_segment_complete(segment: CookingSegment, idx: int):
        print(f"[CALLBACK] ✓ Segment {idx + 1} completed (ID: {segment.segment_id})")
    
    def on_recipe_complete(recipe: Recipe):
        print(f"\n[CALLBACK] ✓✓✓ Recipe '{recipe.name}' completed successfully! ✓✓✓")
    
    def on_error(error: Exception):
        print(f"[CALLBACK] ✗ Error occurred: {error}")
    
    # Create engine and test recipe
    engine = CookingEngine()
    test_recipe = create_test_recipe()
    
    print("="*60)
    print("EPICURA COOKING ENGINE TEST")
    print("="*60)
    print(f"Recipe: {test_recipe.name}")
    print(f"Segments: {test_recipe.segment_count}")
    print(f"Total Time: {test_recipe.cooking_time}s")
    print(f"Schedule: {test_recipe.segment_schedule}")
    print(f"Ingredient Types: {[t.value for t in test_recipe.ingredient_types]}")
    print("="*60)
    
    # Start cooking
    engine.cook_recipe(
        test_recipe,
        on_segment_start=on_segment_start,
        on_segment_complete=on_segment_complete,
        on_recipe_complete=on_recipe_complete,
        on_error=on_error
    )
    
    # Monitor status periodically
    try:
        while engine.is_cooking:
            status = engine.get_status()
            print(f"\n[STATUS] Progress: {status['progress_percent']:.1f}% | "
                  f"Elapsed: {status['elapsed_time']:.0f}s | "
                  f"Remaining: {status['remaining_time']:.0f}s | "
                  f"Segment: {status['current_segment_index'] + 1 if status['current_segment_index'] is not None else 'N/A'}")
            time.sleep(10)  # Update every 10 seconds
            
    except KeyboardInterrupt:
        print("\n\n[INTERRUPTED] Stopping cooking due to user interrupt...")
        engine.stop_cooking()
