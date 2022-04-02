package it.units.erallab.evolution.builder;

import it.units.erallab.hmsrobots.util.Grid;

import java.util.Collections;
import java.util.List;
import java.util.function.Function;

/**
 * @author eric on 2021/01/01 for VSREvolution
 */
public class DirectNumbersGrid implements PrototypedFunctionBuilder<List<Double>, Grid<double[]>> {

  @Override
  public Function<List<Double>, Grid<double[]>> buildFor(Grid<double[]> grid) {
    int expectedLength = grid.values().stream().mapToInt(v -> v.length).sum();
    return values -> {
      if (expectedLength != values.size()) {
        throw new IllegalArgumentException(String.format(
            "Wrong number of values: %d expected, %d found",
            expectedLength,
            values.size()
        ));
      }
      int c = 0;
      Grid<double[]> output = Grid.create(grid);
      for (int x = 0; x < output.getW(); x++) {
        for (int y = 0; y < output.getH(); y++) {
          double[] local = values.subList(c, c + grid.get(x, y).length).stream().mapToDouble(d -> d).toArray();
          c = c + grid.get(x, y).length;
          output.set(x, y, local);
        }
      }
      return output;
    };
  }

  @Override
  public List<Double> exampleFor(Grid<double[]> grid) {
    return Collections.nCopies(
        grid.values().stream().mapToInt(v -> v.length).sum(),
        0d
    );
  }
}
