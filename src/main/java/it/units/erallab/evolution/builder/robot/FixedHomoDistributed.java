package it.units.erallab.evolution.builder.robot;

import it.units.erallab.evolution.builder.PrototypedFunctionBuilder;
import it.units.erallab.hmsrobots.core.controllers.DistributedSensing;
import it.units.erallab.hmsrobots.core.controllers.RealFunction;
import it.units.erallab.hmsrobots.core.controllers.TimedRealFunction;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.core.objects.SensingVoxel;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.util.SerializationUtils;

import java.util.List;
import java.util.Objects;
import java.util.function.Function;
import java.util.stream.Collectors;

/**
 * @author eric
 */
public class FixedHomoDistributed implements PrototypedFunctionBuilder<TimedRealFunction, Robot<? extends SensingVoxel>> {
  private final int signals;

  public FixedHomoDistributed(int signals) {
    this.signals = signals;
  }

  @Override
  public Function<TimedRealFunction, Robot<? extends SensingVoxel>> buildFor(Robot<? extends SensingVoxel> robot) {
    int[] dim = getIODim(robot);
    Grid<? extends SensingVoxel> body = robot.getVoxels();
    int nOfInputs = dim[0];
    int nOfOutputs = dim[1];
    return function -> {
      if (function.getInputDimension() != nOfInputs) {
        throw new IllegalArgumentException(String.format(
            "Wrong number of function input args: %d expected, %d found",
            nOfInputs,
            function.getInputDimension()
        ));
      }
      if (function.getOutputDimension() != nOfOutputs) {
        throw new IllegalArgumentException(String.format(
            "Wrong number of function output args: %d expected, %d found",
            nOfOutputs,
            function.getOutputDimension()
        ));
      }
      DistributedSensing controller = new DistributedSensing(body, signals);
      for (Grid.Entry<? extends SensingVoxel> entry : body) {
        if (entry.getValue() != null) {
          controller.getFunctions().set(entry.getX(), entry.getY(), SerializationUtils.clone(function));
        }
      }
      return new Robot<>(
          controller,
          SerializationUtils.clone(body)
      );
    };
  }

  @Override
  public TimedRealFunction exampleFor(Robot<? extends SensingVoxel> robot) {
    int[] dim = getIODim(robot);
    return RealFunction.build(d -> d, dim[0], dim[1]);
  }

  private int[] getIODim(Robot<? extends SensingVoxel> robot) {
    Grid<? extends SensingVoxel> body = robot.getVoxels();
    SensingVoxel voxel = body.values().stream().filter(Objects::nonNull).findFirst().orElse(null);
    if (voxel == null) {
      throw new IllegalArgumentException("Target robot has no voxels");
    }
    int nOfInputs = DistributedSensing.nOfInputs(voxel, signals);
    int nOfOutputs = DistributedSensing.nOfOutputs(voxel, signals);
    List<Grid.Entry<? extends SensingVoxel>> wrongVoxels = body.stream()
        .filter(e -> e.getValue() != null)
        .filter(e -> DistributedSensing.nOfInputs(e.getValue(), signals) != nOfInputs)
        .collect(Collectors.toList());
    if (!wrongVoxels.isEmpty()) {
      throw new IllegalArgumentException(String.format(
          "Cannot build %s robot mapper for this body: all voxels should have %d inputs, but voxels at positions %s have %s",
          getClass().getSimpleName(),
          nOfInputs,
          wrongVoxels.stream()
              .map(e -> String.format("(%d,%d)", e.getX(), e.getY()))
              .collect(Collectors.joining(",")),
          wrongVoxels.stream()
              .map(e -> String.format("%d", DistributedSensing.nOfInputs(e.getValue(), signals)))
              .collect(Collectors.joining(","))
      ));
    }
    return new int[]{nOfInputs, nOfOutputs};
  }

}
