"""
main.py
Epicura Cooking System - Main entry point demonstrating CookingScheduler and CookingBot usage
"""

import time
from RecipeModel import *
from CookingBot import CookingBot
from CookingScheduler import CookingScheduler


def create_sample_recipe() -> Recipe:
    """
    Create a sample recipe for testing the cooking system.
    
    Returns:
        Recipe object with multiple cooking segments
    """
    
    # Segment 1: Heat oil and add spices (30 seconds)
    segment1 = CookingSegment(
        segment_id=1,
        duration=30,
        ingredients={
            IngredientType.SLD: IngredientVector(index=0, quantity=50, unit="ml"),  # Oil
            IngredientType.ASD: IngredientVector(index=1, quantity=5, unit="g"),    # Cumin
            IngredientType.ASD: IngredientVector(index=2, quantity=3, unit="g")     # Turmeric
        },
        ISC=[
            TempProfile(
                temp_start=25, 
                temp_end=180, 
                duration=30, 
                profile_type=TemperatureProfileType.LINEAR
            )
        ],
        stirring_profile=[
            StirringProfile(
                duration=30, 
                rpm=200, 
                profile_type=StirringProfileType.CLOCKWISE_CONSTANT_RPM
            )
        ]
    )
    
    # Segment 2: Add vegetables and sauté (45 seconds)
    segment2 = CookingSegment(
        segment_id=2,
        duration=45,
        ingredients={
            IngredientType.CID: IngredientVector(index=0, quantity=200, unit="g"),  # Onions
            IngredientType.CID: IngredientVector(index=1, quantity=100, unit="g"),  # Bell peppers
            IngredientType.LID: IngredientVector(index=0, quantity=150, unit="g")   # Tomatoes
        },
        ISC=[
            TempProfile(
                temp_start=180, 
                temp_end=180, 
                duration=45,
                profile_type=TemperatureProfileType.STEPWISE
            )
        ],
        stirring_profile=[
            StirringProfile(
                duration=45, 
                rpm=150,
                profile_type=StirringProfileType.CLOCKWISE_INTERMITTENT_RPM
            )
        ]
    )
    
    # Segment 3: Add liquid and simmer (60 seconds)
    segment3 = CookingSegment(
        segment_id=3,
        duration=60,
        ingredients={
            IngredientType.SLD: IngredientVector(index=1, quantity=200, unit="ml"),  # Water
            IngredientType.ASD: IngredientVector(index=5, quantity=2, unit="g"),     # Garam masala
            IngredientType.VLD: IngredientVector(index=0, quantity=50, unit="ml")    # Cream
        },
        ISC=[
            TempProfile(
                temp_start=180, 
                temp_end=95, 
                duration=60,
                profile_type=TemperatureProfileType.LINEAR
            )
        ],
        stirring_profile=[
            StirringProfile(
                duration=60, 
                rpm=100,
                profile_type=StirringProfileType.ALTERNATING_CONSTANT_RPM
            )
        ]
    )
    
    # Segment 4: Final simmer (45 seconds)
    segment4 = CookingSegment(
        segment_id=4,
        duration=45,
        ingredients={},  # No ingredients added in this segment
        ISC=[
            TempProfile(
                temp_start=95, 
                temp_end=85, 
                duration=45,
                profile_type=TemperatureProfileType.EXPONENTIAL
            )
        ],
        stirring_profile=[
            StirringProfile(
                duration=45, 
                rpm=80,
                profile_type=StirringProfileType.CLOCKWISE_CONSTANT_RPM
            )
        ]
    )
    
    recipe = Recipe(
        id="recipe_001",
        name="Vegetable Curry",
        cooking_segments=[segment1, segment2, segment3, segment4]
    )
    
    return recipe


# Callback functions for cooking events
def on_segment_start(segment: CookingSegment, idx: int):
    """Called when a segment starts."""
    print(f"\n🎯 [CALLBACK] Segment {idx + 1} started (ID: {segment.segment_id})")
    print(f"   Ingredients: {len(segment.ingredients)}, "
          f"Temp profiles: {len(segment.ISC)}, "
          f"Stirring profiles: {len(segment.stirring_profile)}")


def on_segment_complete(segment: CookingSegment, idx: int):
    """Called when a segment completes."""
    print(f"✅ [CALLBACK] Segment {idx + 1} completed (ID: {segment.segment_id})")


def on_recipe_complete(recipe: Recipe):
    """Called when entire recipe is complete."""
    print(f"\n{'='*60}")
    print(f"🎉 [CALLBACK] Recipe '{recipe.name}' completed successfully!")
    print(f"   Total segments executed: {recipe.segment_count}")
    print(f"   Total cooking time: {recipe.cooking_time}s")
    print(f"{'='*60}\n")


def on_error(error: Exception):
    """Called if an error occurs during cooking."""
    print(f"\n❌ [CALLBACK] Error occurred: {error}")


def print_recipe_summary(recipe: Recipe):
    """Print a summary of the recipe."""
    print(f"\n{'='*60}")
    print(f"📖 RECIPE SUMMARY")
    print(f"{'='*60}")
    print(f"Name: {recipe.name}")
    print(f"ID: {recipe.id}")
    print(f"Total Segments: {recipe.segment_count}")
    print(f"Total Cooking Time: {recipe.cooking_time}s ({recipe.cooking_time // 60}m {recipe.cooking_time % 60}s)")
    print(f"Total Ingredients: {recipe.ingredient_count}")
    print(f"Ingredient Types: {', '.join([t.value for t in recipe.ingredient_types])}")
    print(f"Segment Schedule: {recipe.segment_schedule}")
    print(f"{'='*60}\n")


def main():
    """Main entry point for the Epicura cooking system."""
    
    print("\n" + "="*60)
    print("🍳 EPICURA AUTONOMOUS COOKING SYSTEM")
    print("="*60 + "\n")
    
    # Step 1: Create the CookingBot (handles hardware)
    print("Step 1: Initializing CookingBot...")
    cooking_bot = CookingBot()
    cooking_bot.initialize_hardware()
    print()
    
    # Step 2: Create the CookingScheduler (handles timing)
    print("Step 2: Initializing CookingScheduler...")
    scheduler = CookingScheduler(cooking_bot)
    print("✅ CookingScheduler initialized\n")
    
    # Step 3: Create a recipe
    print("Step 3: Loading recipe...")
    recipe = create_sample_recipe()
    print_recipe_summary(recipe)
    
    # Step 4: Start cooking
    print("Step 4: Starting cooking process...")
    print("(Press Ctrl+C to stop cooking)\n")
    
    try:
        # Start the cooking process
        scheduler.cook_recipe(
            recipe,
            on_segment_start=on_segment_start,
            on_segment_complete=on_segment_complete,
            on_recipe_complete=on_recipe_complete,
            on_error=on_error
        )
        
        # Monitor status periodically while cooking
        last_status_time = time.time()
        status_interval = 15  # Print status every 15 seconds
        
        while scheduler.is_cooking:
            current_time = time.time()
            
            # Print status update at regular intervals
            if current_time - last_status_time >= status_interval:
                status = scheduler.get_status()
                bot_status = cooking_bot.get_hardware_status()
                
                print(f"\n{'─'*60}")
                print(f"📊 STATUS UPDATE")
                print(f"{'─'*60}")
                print(f"Recipe: {status['current_recipe']}")
                print(f"Progress: {status['progress_percent']:.1f}%")
                print(f"Elapsed: {status['elapsed_time']:.0f}s")
                print(f"Remaining: {status['remaining_time']:.0f}s")
                print(f"Current Segment: {status['current_segment_index'] + 1 if status['current_segment_index'] is not None else 'N/A'}/{recipe.segment_count}")
                print(f"Temperature: {bot_status['current_temperature']}°C")
                print(f"Stirring: {'Yes' if bot_status['is_stirring'] else 'No'} "
                      f"({bot_status['stirring_rpm']} RPM)")
                print(f"{'─'*60}")
                
                last_status_time = current_time
            
            time.sleep(1)  # Check every second
        
        # Cooking completed
        print("\n✅ Cooking process completed successfully!")
        
    except KeyboardInterrupt:
        print("\n\n⚠️  [INTERRUPTED] User interrupted cooking process")
        print("Initiating graceful shutdown...")
        scheduler.stop_cooking()
        print("✅ Shutdown complete")
        
    except Exception as e:
        print(f"\n❌ [ERROR] An error occurred: {e}")
        print("Initiating emergency shutdown...")
        scheduler.stop_cooking()
        print("✅ Emergency shutdown complete")
    
    finally:
        # Final hardware status
        final_status = cooking_bot.get_hardware_status()
        print(f"\n{'='*60}")
        print(f"🔧 FINAL HARDWARE STATUS")
        print(f"{'='*60}")
        print(f"Active: {final_status['is_active']}")
        print(f"Temperature: {final_status['current_temperature']}°C")
        print(f"Stirring: {final_status['is_stirring']}")
        print(f"{'='*60}\n")


if __name__ == "__main__":
    main()
