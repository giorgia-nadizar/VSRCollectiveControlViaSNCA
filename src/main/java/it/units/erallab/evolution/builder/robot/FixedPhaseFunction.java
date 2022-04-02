package it.units.erallab.evolution.builder.robot;

import it.units.erallab.evolution.builder.PrototypedFunctionBuilder;
import it.units.erallab.hmsrobots.core.controllers.PhaseSin;
import it.units.erallab.hmsrobots.core.controllers.RealFunction;
import it.units.erallab.hmsrobots.core.controllers.TimedRealFunction;
import it.units.erallab.hmsrobots.core.objects.ControllableVoxel;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.util.SerializationUtils;

import java.util.function.Function;

/**
 * @author eric
 */
public class FixedPhaseFunction implements PrototypedFunctionBuilder<TimedRealFunction, Robot<?>> {
  private final double frequency;
  private final double amplitude;

  public FixedPhaseFunction(double frequency, double amplitude) {
    this.frequency = frequency;
    this.amplitude = amplitude;
  }

  @Override
  public Function<TimedRealFunction, Robot<?>> buildFor(Robot<?> robot) {
    Grid<? extends ControllableVoxel> body = robot.getVoxels();
    return function -> {
      if (function.getInputDimension() != 2) {
        throw new IllegalArgumentException(String.format(
            "Wrong number of function input args: 2 expected, %d found",
            function.getInputDimension()
        ));
      }
      return new Robot<>(
          new PhaseSin(
              frequency,
              amplitude,
              Grid.create(
                  body.getW(),
                  body.getH(),
                  (x, y) -> function.apply(0d, new double[]{
                      (double) x / (double) body.getW(),
                      (double) y / (double) body.getW()}
                  )[0]
              )
          ),
          SerializationUtils.clone(body)
      );
    };
  }

  @Override
  public TimedRealFunction exampleFor(Robot<?> robot) {
    return RealFunction.build(d -> d, 2, 1);
  }

}
