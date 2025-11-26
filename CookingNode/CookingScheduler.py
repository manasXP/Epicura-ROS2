"""
CookingScheduler.py
Epicura Cooking Scheduler - Manages timing and scheduling of cooking segments
"""

import time
import threading
from typing import Callable, Optional
from datetime import datetime, timedelta
from RecipeModel import Recipe, CookingSegment


class CookingScheduler:
    """
    Manages the scheduling and timing of recipe execution.
    Orchestrates when each cooking segment should start and end based on the recipe schedule.
    """
    
    def __init__(self, cooking_bot):
        """
        Initialize the cooking scheduler.
        
        Args:
            cooking_bot: Instance of CookingBot to execute cooking operations
        """
        self.cooking_bot = cooking_bot
        self.is_cooking = False
        self.current_segment = None
        self.current_segment_index = None
        self.current_recipe = None
        self.cooking_thread = None
        self.stop_event = threading.Event()
        self.recipe_start_time = None
        
        # Callbacks
        self.on_segment_start = None
        self.on_segment_complete = None
        self.on_recipe_complete = None
        self.on_error = None
        
    def cook_recipe(self, recipe: Recipe, 
                    on_segment_start: Optional[Callable[[CookingSegment, int], None]] = None,
                    on_segment_complete: Optional[Callable[[CookingSegment, int], None]] = None,
                    on_recipe_complete: Optional[Callable[[Recipe], None]] = None,
                    on_error: Optional[Callable[[Exception], None]] = None):
        """
        Schedule and execute a recipe by processing each cooking segment according to segment_schedule.
        
        Args:
            recipe: Recipe object containing cooking segments to process
            on_segment_start: Callback function called when a segment starts (segment, segment_index)
            on_segment_complete: Callback function called when a segment completes (segment, segment_index)
            on_recipe_complete: Callback function called when entire recipe is complete
            on_error: Callback function called if an error occurs during cooking
        """
        if self.is_cooking:
            raise RuntimeError("Scheduler is already cooking a recipe. Stop current recipe first.")
        
        # Store callbacks
        self.on_segment_start = on_segment_start
        self.on_segment_complete = on_segment_complete
        self.on_recipe_complete = on_recipe_complete
        self.on_error = on_error
        
        self.current_recipe = recipe
        self.is_cooking = True
        self.stop_event.clear()
        
        # Start cooking in a separate thread to avoid blocking
        self.cooking_thread = threading.Thread(
            target=self._schedule_recipe_thread,
            args=(recipe,),
            daemon=True
        )
        self.cooking_thread.start()
        
    def _schedule_recipe_thread(self, recipe: Recipe):
        """Internal method that runs in a separate thread to schedule and execute recipe segments."""
        try:
            # Get the segment schedule (cumulative times)
            segment_schedule = recipe.segment_schedule
            
            print(f"\n{'='*60}")
            print(f"🍳 Starting Recipe: {recipe.name}")
            print(f"⏱️  Total Cooking Time: {recipe.cooking_time} seconds ({recipe.cooking_time // 60}m {recipe.cooking_time % 60}s)")
            print(f"📋 Number of Segments: {recipe.segment_count}")
            print(f"📅 Segment Schedule: {segment_schedule}")
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
                    print(f"⏳ [TIMING] Waiting {wait_time:.1f}s to maintain schedule...")
                    time.sleep(wait_time)
                    elapsed_time = time.time() - self.recipe_start_time
                
                # Check if we're behind schedule
                if elapsed_time > segment_start_time + 1.0:  # 1 second tolerance
                    delay = elapsed_time - segment_start_time
                    print(f"⚠️  [WARNING] Segment {idx + 1} starting {delay:.1f}s behind schedule")
                
                # Call segment start callback
                if self.on_segment_start:
                    self.on_segment_start(segment, idx)
                
                # Execute the segment with scheduled timing
                self._execute_segment_scheduled(segment, idx, segment_start_time, segment_end_time)
                
                # Call segment complete callback
                if self.on_segment_complete:
                    self.on_segment_complete(segment, idx)
            
            # Recipe completed successfully
            if not self.stop_event.is_set():
                total_time = time.time() - self.recipe_start_time
                print(f"\n{'='*60}")
                print(f"✅ Recipe Complete: {recipe.name}")
                print(f"⏱️  Actual Total Time: {total_time:.1f}s")
                print(f"📋 Scheduled Time: {recipe.cooking_time}s")
                print(f"📊 Time Variance: {total_time - recipe.cooking_time:+.1f}s")
                print(f"{'='*60}\n")
                
                if self.on_recipe_complete:
                    self.on_recipe_complete(recipe)
                    
        except Exception as e:
            print(f"\n❌ [ERROR] Cooking error: {e}")
            if self.on_error:
                self.on_error(e)
        finally:
            self.is_cooking = False
            self.current_segment = None
            self.current_segment_index = None
            self.current_recipe = None
            self.recipe_start_time = None
    
    def _execute_segment_scheduled(self, segment: CookingSegment, segment_index: int,
                                   scheduled_start: float, scheduled_end: float):
        """
        Execute a single cooking segment with precise scheduled timing.
        
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
        print(f"📍 Segment {segment_index + 1} of {len(self.current_recipe.cooking_segments)} (ID: {segment.segment_id})")
        print(f"⏱️  Duration: {segment_duration}s ({segment_duration // 60}m {segment_duration % 60:.0f}s)")
        print(f"📅 Scheduled: {scheduled_start}s → {scheduled_end}s (from recipe start)")
        print(f"🕐 Clock Time: {start_datetime.strftime('%H:%M:%S')} → {end_datetime.strftime('%H:%M:%S')}")
        print(f"{'─'*60}")
        
        # Record segment start time
        segment_actual_start = time.time()
        target_segment_end = self.recipe_start_time + scheduled_end
        
        # Tell the CookingBot to execute this segment
        self.cooking_bot.execute_segment(segment, segment_duration, self.stop_event)
        
        # Wait for the segment to complete according to schedule
        while time.time() < target_segment_end:
            if self.stop_event.is_set():
                break
            
            # Calculate progress
            elapsed = time.time() - segment_actual_start
            progress = (elapsed / segment_duration) * 100
            remaining = target_segment_end - time.time()
            
            # Print progress every 10 seconds
            if int(elapsed) % 10 == 0 and elapsed > 0 and elapsed < segment_duration:
                print(f"  📊 [Progress] {progress:.1f}% complete, {remaining:.0f}s remaining")
                time.sleep(1)  # Prevent multiple prints in the same second
            
            time.sleep(0.1)  # Check every 100ms for responsiveness
        
        # Tell the CookingBot to finish the segment
        self.cooking_bot.finish_segment(segment)
        
        actual_duration = time.time() - segment_actual_start
        variance = actual_duration - segment_duration
        
        print(f"\n  ✅ Segment {segment_index + 1} Complete")
        print(f"  ⏱️  Actual Duration: {actual_duration:.1f}s (variance: {variance:+.1f}s)")
        print(f"  🕐 End Time: {datetime.now().strftime('%H:%M:%S')}\n")
    
    def stop_cooking(self):
        """Stop the current cooking process gracefully."""
        if self.is_cooking:
            print("\n🛑 [STOPPING] Stopping cooking process...")
            self.stop_event.set()
            
            # Tell the CookingBot to perform emergency stop
            self.cooking_bot.emergency_stop()
            
            if self.cooking_thread:
                self.cooking_thread.join(timeout=5.0)
            
            self.is_cooking = False
            self.current_segment = None
            self.current_segment_index = None
            self.current_recipe = None
            self.recipe_start_time = None
            print("✅ [STOPPED] Cooking process stopped successfully")
    
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
    
    def get_elapsed_time(self) -> float:
        """Get elapsed time in seconds since recipe started."""
        if not self.is_cooking or not self.recipe_start_time:
            return 0
        
        return time.time() - self.recipe_start_time
