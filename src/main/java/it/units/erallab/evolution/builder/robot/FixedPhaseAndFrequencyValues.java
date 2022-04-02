package it.units.erallab.evolution.builder.robot;

import it.units.erallab.evolution.builder.PrototypedFunctionBuilder;
import it.units.erallab.hmsrobots.core.controllers.TimeFunctions;
import it.units.erallab.hmsrobots.core.objects.ControllableVoxel;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.util.SerializableFunction;
import it.units.erallab.hmsrobots.util.SerializationUtils;

import java.util.Collections;
import java.util.List;
import java.util.Objects;
import java.util.function.Function;

/**
 * @author eric
 */
public class FixedPhaseAndFrequencyValues implements PrototypedFunctionBuilder<List<Double>, Robot<?>> {

  private final double amplitude;

  public FixedPhaseAndFrequencyValues(double amplitude) {
    this.amplitude = amplitude;
  }

  @Override
  public Function<List<Double>, Robot<?>> buildFor(Robot<?> robot) {
    Grid<? extends ControllableVoxel> body = robot.getVoxels();
    long nOfVoxel = body.values().stream().filter(Objects::nonNull).count();
    return values -> {
      if (2 * nOfVoxel != values.size()) {
        throw new IllegalArgumentException(String.format(
            "Wrong number of values: %d expected, %d found",
            nOfVoxel,
            values.size()
        ));
      }
      int c = 0;
      Grid<SerializableFunction<Double, Double>> functions = Grid.create(body);
      for (Grid.Entry<?> entry : body) {
        if (entry.getValue() != null) {
          double f = values.get(c);
          double phi = values.get(c + 1);
          double finalAmplitude = amplitude;
          functions.set(entry.getX(), entry.getY(), t -> finalAmplitude * Math.sin(2 * Math.PI * f * t + phi));
          c = c + 2;
        }
      }
      return new Robot<>(
          new TimeFunctions(functions),
          SerializationUtils.clone(body)
      );
    };
  }

  @Override
  public List<Double> exampleFor(Robot<?> robot) {
    long nOfVoxel = robot.getVoxels().values().stream().filter(Objects::nonNull).count();
    return Collections.nCopies((int) nOfVoxel * 2, 0d);
  }

}
