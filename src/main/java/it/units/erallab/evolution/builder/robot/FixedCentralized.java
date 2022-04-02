package it.units.erallab.evolution.builder.robot;

import it.units.erallab.evolution.builder.PrototypedFunctionBuilder;
import it.units.erallab.hmsrobots.core.controllers.CentralizedSensing;
import it.units.erallab.hmsrobots.core.controllers.RealFunction;
import it.units.erallab.hmsrobots.core.controllers.TimedRealFunction;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.core.objects.SensingVoxel;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.util.SerializationUtils;

import java.util.function.Function;

/**
 * @author eric
 */
public class FixedCentralized implements PrototypedFunctionBuilder<TimedRealFunction, Robot<? extends SensingVoxel>> {

  @Override
  public Function<TimedRealFunction, Robot<? extends SensingVoxel>> buildFor(Robot<? extends SensingVoxel> robot) {
    Grid<? extends SensingVoxel> body = robot.getVoxels();
    return function -> {
      if (function.getInputDimension() != CentralizedSensing.nOfInputs(body)) {
        throw new IllegalArgumentException(String.format(
            "Wrong number of function input args: %d expected, %d found",
            CentralizedSensing.nOfInputs(body),
            function.getInputDimension()
        ));
      }
      if (function.getOutputDimension() != CentralizedSensing.nOfOutputs(body)) {
        throw new IllegalArgumentException(String.format(
            "Wrong number of function output args: %d expected, %d found",
            CentralizedSensing.nOfOutputs(body),
            function.getOutputDimension()
        ));
      }
      return new Robot<>(
          new CentralizedSensing(body, function),
          SerializationUtils.clone(body)
      );
    };
  }

  @Override
  public TimedRealFunction exampleFor(Robot<? extends SensingVoxel> robot) {
    Grid<? extends SensingVoxel> body = robot.getVoxels();
    return RealFunction.build(
        d -> d,
        CentralizedSensing.nOfInputs(body),
        CentralizedSensing.nOfOutputs(body)
    );
  }

}
