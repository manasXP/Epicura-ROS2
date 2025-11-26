from dataclasses import dataclass
from typing import List, Dict, Optional
from enum import Enum

class IngredientType(Enum):
    ASD = "Advanced Spice Dispenser"
    CID = "Coarse Ingredient Dispenser"
    SLD = "Standard Liquid Dispenser"
    VLD = "Viscious Liquid Dispenser"
    LID = "Lumped Ingredient Dispenser"
    UNCLASSIFIED = "Unclassified Ingredient"

class TemperatureProfileType(Enum):
    LINEAR = "Linear"
    STEPWISE = "Stepwise"
    EXPONENTIAL = "Exponential"
    QUADRATIC = "Quadratic"
    UNKNOWN = "Unknown"

class StirringProfileType(Enum):
    CLOCKWISE_CONSTANT_RPM = "Clockwise Constant RPM"
    CLOCKWISE_VARIABLE_RPM = "Clockwise Variable RPM"
    CLOCKWISE_INTERMITTENT_RPM = "Clockwise Intermittent RPM"
    COUNTERCLOCKWISE_CONSTANT_RPM = "CounterClockwise Constant RPM"
    COUNTERCLOCKWISE_VARIABLE_RPM = "CounterClockwise Variable RPM"
    COUNTERCLOCKWISE_INTERMITTENT_RPM = "CounterClockwise Intermittent RPM"
    ALTERNATING_CONSTANT_RPM = "Alternating Constant RPM"
    ALTERNATING_VARIABLE_RPM = "Alternating Variable RPM"
    UNKNOWN = "Unknown"

@dataclass
class IngredientVector:
    index: int
    quantity: int
    unit: str

@dataclass
class TempProfile:
    temp_start: int  # Centigrade
    temp_end: int    # Centigrade
    duration: int    # Seconds
    profile_type: TemperatureProfileType

@dataclass
class StirringProfile:
    duration: int  # Seconds
    rpm: int  # Revolutions per minute (FIXED: was "int." with a period)
    profile_type: StirringProfileType

@dataclass
class CookingSegment:
    segment_id: int
    duration: int  # Seconds
    ingredients: Dict[IngredientType, IngredientVector]
    ISC: List[TempProfile]
    stirring_profile: List[StirringProfile]

@dataclass
class Recipe:
    name: str
    cooking_segments: List[CookingSegment]
    id: Optional[str] = None
    
    @property
    def cooking_time(self) -> int:
        return sum(segment.duration for segment in self.cooking_segments)

    @property
    def ingredient_count(self) -> int:
        count = 0
        for segment in self.cooking_segments:
            count += sum(vector.quantity for vector in segment.ingredients.values())
        return count

    @property
    def segment_count(self) -> int:
        return len(self.cooking_segments)
    
    @property
    def ingredient_types(self) -> List[IngredientType]:
        types = set()
        for segment in self.cooking_segments:
            types.update(segment.ingredients.keys())
        return list(types)
    
    @property
    def segment_schedule(self) -> List[int]:
        schedule = []
        cumulative_time = 0
        for segment in self.cooking_segments:
            cumulative_time += segment.duration
            schedule.append(cumulative_time)
        return schedule
