package it.units.erallab.evolution.builder;

import it.units.erallab.hmsrobots.core.controllers.TimedRealFunction;
import it.units.erallab.hmsrobots.util.Grid;

import java.util.Collections;
import java.util.List;
import java.util.Objects;
import java.util.function.Function;

/**
 * @author eric
 */
public class FunctionGrid implements PrototypedFunctionBuilder<List<Double>, Grid<TimedRealFunction>> {

  private final PrototypedFunctionBuilder<List<Double>, TimedRealFunction> itemBuilder;

  public FunctionGrid(PrototypedFunctionBuilder<List<Double>, TimedRealFunction> itemBuilder) {
    this.itemBuilder = itemBuilder;
  }

  @Override
  public Function<List<Double>, Grid<TimedRealFunction>> buildFor(Grid<TimedRealFunction> targetFunctions) {
    return values -> {
      Grid<TimedRealFunction> functions = Grid.create(targetFunctions);
      int c = 0;
      for (Grid.Entry<TimedRealFunction> entry : targetFunctions) {
        if (entry.getValue() == null) {
          continue;
        }
        int size = itemBuilder.exampleFor(entry.getValue()).size();
        functions.set(
            entry.getX(),
            entry.getY(),
            itemBuilder.buildFor(entry.getValue()).apply(values.subList(c, c + size))
        );
        c = c + size;
      }
      return functions;
    };
  }

  @Override
  public List<Double> exampleFor(Grid<TimedRealFunction> functions) {
    return Collections.nCopies(
        functions.values().stream()
            .filter(Objects::nonNull)
            .mapToInt(f -> itemBuilder.exampleFor(f).size())
            .sum(),
        0d
    );
  }

}
