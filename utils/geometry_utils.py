from pydrake.all import *


class PoseTransform(LeafSystem):
    def __init__(
        self,
        X_BA: RigidTransform = RigidTransform(),
    ):
        LeafSystem.__init__(self)
        self.DeclareAbstractInputPort("pose", AbstractValue.Make(RigidTransform()))
        self.DeclareAbstractOutputPort(
            "pose",
            lambda: AbstractValue.Make(RigidTransform()),
            self._CalcOutput,
        )
        self.X_BA = X_BA

    def _CalcOutput(self, context, output):
        pose = self.EvalAbstractInput(context, 0).get_value()
        pose = pose @ self.X_BA
        output.get_mutable_value().set(pose.rotation(), pose.translation())
