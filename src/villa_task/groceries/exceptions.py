class GraspException(Exception):
    pass


class PreGraspPlanFailure(GraspException):
    pass


class MidGraspPlanFailure(GraspException):
    pass


class ItemNotGraspedFailure(GraspException):
    pass

class PlaceException(Exception):
    pass