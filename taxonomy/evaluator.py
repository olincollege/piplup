from abc import ABC, abstractmethod
from typing import Tuple, Any
from dataclasses import dataclass
from taxonomy.properties import *

@dataclass
class GraspabilityScore:
    score : float = 0
    confidence : float = 0

class Evaluator(ABC):

    @abstractmethod
    def evaluate(self, property: Any, score : GraspabilityScore) -> GraspabilityScore:
        pass

    @abstractmethod
    def should_evaluate(self, score : GraspabilityScore) -> bool:
        pass



class PlanarEvaluator(Evaluator):
    def evaluate(self, property: Any, score : GraspabilityScore) -> GraspabilityScore:
        score.score = 1
        score.confidence = 1
        return score

    def should_evaluate(self, score : GraspabilityScore) -> bool:
        return True

class SuctionEvaluator(Evaluator):
    def evaluate(self, property: Any, score : GraspabilityScore) -> GraspabilityScore:
        score.score = 1
        score.confidence = 1
        return score

    def should_evaluate(self, score : GraspabilityScore) -> bool:
        return True