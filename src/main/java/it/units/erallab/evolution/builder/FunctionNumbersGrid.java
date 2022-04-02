package it.units.erallab.evolution.builder;

import it.units.erallab.hmsrobots.core.controllers.RealFunction;
import it.units.erallab.hmsrobots.util.Grid;

import java.util.Arrays;
import java.util.function.Function;

/**
 * @author eric on 2021/01/01 for VSREvolution
 */
public class FunctionNumbersGrid implements PrototypedFunctionBuilder<RealFunction, Grid<double[]>> {

  @Override
  public Function<RealFunction, Grid<double[]>> buildFor(Grid<double[]> grid) {
    int targetLength = targetLength(grid);
    return function -> {
      if (function.getInputDimension() != 2 || function.getOutputDimension() != targetLength) {
        throw new IllegalArgumentException(String.format(
            "Wrong number of body function args: 2->%d expected, %d->%d found",
            targetLength,
            function.getInputDimension(),
            function.getOutputDimension()
        ));
      }
      Grid<double[]> output = Grid.create(grid);
      double w = output.getW();
      double h = output.getH();
      for (double x = 0; x < output.getW(); x++) {
        for (double y = 0; y < output.getH(); y++) {
          output.set((int) x, (int) y, function.apply(new double[]{x / (w - 1d), y / (h - 1d)}));
        }
      }
      return output;
    };
  }

  @Override
  public RealFunction exampleFor(Grid<double[]> grid) {
    int targetLength = targetLength(grid);
    return RealFunction.build(d -> d, 2, targetLength);
  }

  private static int targetLength(Grid<double[]> grid) {
    int[] lengths = grid.values().stream().mapToInt(v -> v.length).distinct().toArray();
    if (lengths.length > 1) {
      throw new IllegalArgumentException(String.format(
          "Target grid elements must have all the same size, found % distinct sizes %s",
          lengths.length,
          Arrays.toString(lengths)
      ));
    }
    return lengths[0];
  }
}
