package it.units.erallab.evolution.builder.phenotype;

import it.units.erallab.evolution.builder.PrototypedFunctionBuilder;
import it.units.erallab.hmsrobots.core.controllers.MultiLayerPerceptron;
import it.units.erallab.hmsrobots.core.controllers.PruningMultiLayerPerceptron;
import it.units.erallab.hmsrobots.core.controllers.TimedRealFunction;

import java.util.Collections;
import java.util.List;
import java.util.function.Function;

/**
 * @author eric
 */
public class PruningMLP implements PrototypedFunctionBuilder<List<Double>, TimedRealFunction> {

  private final double innerLayerRatio;
  private final int nOfInnerLayers;
  private final MultiLayerPerceptron.ActivationFunction activationFunction;
  private final double pruningTime;
  private final double rate;
  private final PruningMultiLayerPerceptron.Context context;
  private final PruningMultiLayerPerceptron.Criterion criterion;

  public PruningMLP(double innerLayerRatio, int nOfInnerLayers, MultiLayerPerceptron.ActivationFunction activationFunction, double pruningTime, double rate, PruningMultiLayerPerceptron.Context context, PruningMultiLayerPerceptron.Criterion criterion) {
    this.innerLayerRatio = innerLayerRatio;
    this.nOfInnerLayers = nOfInnerLayers;
    this.activationFunction = activationFunction;
    this.pruningTime = pruningTime;
    this.rate = rate;
    this.context = context;
    this.criterion = criterion;
  }

  private int[] innerNeurons(int nOfInputs, int nOfOutputs) {
    int[] innerNeurons = new int[nOfInnerLayers];
    int centerSize = (int) Math.max(2, Math.round(nOfInputs * innerLayerRatio));
    if (nOfInnerLayers > 1) {
      for (int i = 0; i < nOfInnerLayers / 2; i++) {
        innerNeurons[i] = nOfInputs + (centerSize - nOfInputs) / (nOfInnerLayers / 2 + 1) * (i + 1);
      }
      for (int i = nOfInnerLayers / 2; i < nOfInnerLayers; i++) {
        innerNeurons[i] = centerSize + (nOfOutputs - centerSize) / (nOfInnerLayers / 2 + 1) * (i - nOfInnerLayers / 2);
      }
    } else if (nOfInnerLayers > 0) {
      innerNeurons[0] = centerSize;
    }
    return innerNeurons;
  }

  @Override
  public Function<List<Double>, TimedRealFunction> buildFor(TimedRealFunction function) {
    return values -> {
      int nOfInputs = function.getInputDimension();
      int nOfOutputs = function.getOutputDimension();
      int[] innerNeurons = innerNeurons(nOfInputs, nOfOutputs);
      int nOfWeights = MultiLayerPerceptron.countWeights(nOfInputs, innerNeurons, nOfOutputs);
      if (nOfWeights != values.size()) {
        throw new IllegalArgumentException(String.format(
            "Wrong number of values for weights: %d expected, %d found",
            nOfWeights,
            values.size()
        ));
      }
      return new PruningMultiLayerPerceptron(
          activationFunction,
          nOfInputs,
          innerNeurons,
          nOfOutputs,
          values.stream().mapToDouble(d -> d).toArray(),
          pruningTime,
          context,
          criterion,
          rate
      );
    };
  }

  @Override
  public List<Double> exampleFor(TimedRealFunction function) {
    return Collections.nCopies(
        MultiLayerPerceptron.countWeights(
            MultiLayerPerceptron.countNeurons(
                function.getInputDimension(),
                innerNeurons(function.getInputDimension(), function.getOutputDimension()),
                function.getOutputDimension())
        ),
        0d
    );
  }

}
