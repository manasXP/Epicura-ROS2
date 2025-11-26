# Epicura Autonomous Cooking System

A distributed cooking automation platform with precise timing control and hardware integration.

## Architecture

The system is separated into two main components for clean separation of concerns:

### 1. **CookingScheduler** (`CookingScheduler.py`)
**Responsibility**: Timing, scheduling, and orchestration

- Manages recipe execution timeline
- Coordinates segment transitions based on `segment_schedule`
- Maintains precise timing synchronization
- Handles callbacks and event notifications
- Monitors cooking progress and provides status updates

**Key Features**:
- Uses Recipe's `segment_schedule` property for cumulative timing
- Detects and corrects timing drift
- Provides graceful shutdown with stop signals
- Thread-safe operation

### 2. **CookingBot** (`CookingBot.py`)
**Responsibility**: Hardware control and cooking operations

- Controls all cooking subsystems (dispensers, temperature, stirring)
- Manages SPI communication with Arduino M4 and Pi Pico controllers
- Implements safety systems and emergency stop
- Provides hardware status monitoring

**Subsystems Controlled**:
- **ASD** (Advanced Spice Dispenser) - 16 cartridge spice system
- **SLD** (Standard Liquid Dispenser) - Water/oil dispensing with ultrasonic monitoring
- **VLD** (Viscous Liquid Dispenser) - Cream/oil with peristaltic pumps
- **CID** (Coarse Ingredient Dispenser) - Chopped vegetables with load cell
- **LID** (Lumped Ingredient Dispenser) - Chunked items up to 1kg
- **ISC** (Induction Stove Control) - Capacitive touch interface with ToF sensor
- **Stirring System** - Motor control with encoder feedback

## Data Model

### Recipe Structure
```python
Recipe
├── id: str
├── name: str
├── cooking_segments: List[CookingSegment]
└── Properties:
    ├── cooking_time: int (total duration)
    ├── segment_count: int
    ├── ingredient_count: int
    ├── ingredient_types: List[IngredientType]
    └── segment_schedule: List[int]  # Cumulative timing [30, 75, 135, 180]
```

### CookingSegment Structure
```python
CookingSegment
├── segment_id: int
├── duration: int (seconds)
├── ingredients: Dict[IngredientType, IngredientVector]
├── ISC: List[TempProfile]  # Temperature control profiles
└── stirring_profile: List[StirringProfile]
```

## Usage

### Basic Usage
```python
from CookingBot import CookingBot
from CookingScheduler import CookingScheduler
from RecipeModel import Recipe

# Initialize hardware controller
cooking_bot = CookingBot()
cooking_bot.initialize_hardware()

# Initialize scheduler
scheduler = CookingScheduler(cooking_bot)

# Load recipe
recipe = create_your_recipe()

# Start cooking with callbacks
scheduler.cook_recipe(
    recipe,
    on_segment_start=your_callback,
    on_segment_complete=your_callback,
    on_recipe_complete=your_callback,
    on_error=your_error_handler
)

# Monitor status
while scheduler.is_cooking:
    status = scheduler.get_status()
    print(f"Progress: {status['progress_percent']:.1f}%")
    time.sleep(5)
```

### Running the Demo
```bash
python main.py
```

This will run a sample vegetable curry recipe with 4 segments.

## Timing System

The scheduler uses the `segment_schedule` property which provides cumulative timing:

```python
# Example: 3 segments of 30s, 45s, 60s
cooking_segments = [seg1(30s), seg2(45s), seg3(60s)]
segment_schedule = [30, 75, 135]  # Cumulative times

# Segment 1: 0s → 30s
# Segment 2: 30s → 75s
# Segment 3: 75s → 135s
```

### Timing Features:
- **Drift Correction**: If a segment runs long, scheduler adjusts subsequent timing
- **Progress Monitoring**: Real-time progress updates every 10 seconds
- **Variance Tracking**: Reports actual vs scheduled duration for each segment
- **Synchronization**: Waits if ahead of schedule to maintain timing

## Callbacks

The system supports four callback types:

1. **on_segment_start**(segment, index) - Called when segment begins
2. **on_segment_complete**(segment, index) - Called when segment ends
3. **on_recipe_complete**(recipe) - Called when recipe finishes
4. **on_error**(error) - Called if an error occurs

## Hardware Integration

### SPI Communication
The system uses SPI slave architecture with multiple controllers:

```
Raspberry Pi 5 (Master)
├── SPI Bus 0
│   ├── Arduino M4 #1 (ASD + CID + LID)
│   ├── Arduino M4 #2 (SLD + VLD)
│   ├── Pi Pico #1 (ISC - Induction Control)
│   └── Pi Pico #2 (Stirring + Safety)
```

### Temperature Control Profiles
- **LINEAR**: Gradual temperature change at constant rate
- **STEPWISE**: Immediate jump to target temperature
- **EXPONENTIAL**: Exponential heating/cooling curve
- **QUADRATIC**: Quadratic temperature profile

### Stirring Profiles
- **CLOCKWISE_CONSTANT_RPM**: Continuous clockwise stirring
- **CLOCKWISE_INTERMITTENT_RPM**: Periodic clockwise stirring
- **ALTERNATING_CONSTANT_RPM**: Alternating direction stirring
- Plus variable and counterclockwise variants

## Safety Features

1. **Emergency Stop**: Immediate shutdown of all subsystems
2. **LPG Leak Detection**: Continuous monitoring with auto-shutoff
3. **Temperature Monitoring**: Safety limits on all heating elements
4. **Pot Presence Detection**: ToF sensor verification
5. **Level Monitoring**: Ultrasonic sensors prevent overflow

## Status Monitoring

### Scheduler Status
```python
status = scheduler.get_status()
# Returns:
{
    "is_cooking": bool,
    "current_recipe": str,
    "current_segment": int,
    "current_segment_index": int,
    "recipe_id": str,
    "elapsed_time": float,
    "remaining_time": float,
    "progress_percent": float,
    "segment_schedule": List[int]
}
```

### Hardware Status
```python
status = cooking_bot.get_hardware_status()
# Returns:
{
    "is_active": bool,
    "current_segment": int,
    "current_temperature": float,
    "is_stirring": bool,
    "stirring_rpm": int,
    "spi_connected": bool,
    "temperature_controller_connected": bool,
    "stirring_controller_connected": bool
}
```

## Development Roadmap

### Completed ✅
- [x] Data model (RecipeModel.py)
- [x] Scheduling system (CookingScheduler.py)
- [x] Hardware abstraction (CookingBot.py)
- [x] Timing synchronization
- [x] Callback system

### In Progress 🚧
- [ ] SPI controller implementation
- [ ] Induction cooker control (capacitive touch interface)
- [ ] Temperature PID control
- [ ] Stirring motor controller

### Planned 📋
- [ ] Load cell integration for weight measurement
- [ ] Ultrasonic level sensors
- [ ] ToF sensor for pot detection
- [ ] Safety system integration
- [ ] Recipe database and API
- [ ] Web/mobile interface
- [ ] Recipe learning from user corrections

## File Structure

```
epicura/
├── RecipeModel.py          # Data models and recipe structure
├── CookingScheduler.py     # Timing and orchestration
├── CookingBot.py           # Hardware control and operations
├── main.py                 # Example usage and testing
└── README.md               # This file
```

## Contributing

When adding new hardware subsystems:
1. Add interface methods to `CookingBot`
2. Update `emergency_stop()` to include new subsystem
3. Add corresponding data models to `RecipeModel.py` if needed
4. Update hardware status reporting

## Notes

- All times are in seconds
- Temperatures are in Celsius
- Quantities are in grams (g) or milliliters (ml)
- RPM values are revolutions per minute
- SPI communication uses CS pins for device selection

## License

Proprietary - Epicura Autonomous Cooking System
