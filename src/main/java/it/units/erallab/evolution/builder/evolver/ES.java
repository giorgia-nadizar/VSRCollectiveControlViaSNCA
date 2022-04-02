package it.units.erallab.evolution.builder.evolver;

import it.units.erallab.evolution.builder.PrototypedFunctionBuilder;
import it.units.malelab.jgea.core.Individual;
import it.units.malelab.jgea.core.evolver.BasicEvolutionaryStrategy;
import it.units.malelab.jgea.core.evolver.Evolver;
import it.units.malelab.jgea.core.order.PartialComparator;
import it.units.malelab.jgea.representation.sequence.FixedLengthListFactory;
import it.units.malelab.jgea.representation.sequence.numeric.UniformDoubleFactory;

import java.util.List;

/**
 * @author eric
 */
public class ES implements EvolverBuilder<List<Double>> {
  private final double sigma;
  private final int nPop;
  private final boolean remap;

  public ES(double sigma, int nPop, boolean remap) {
    this.sigma = sigma;
    this.nPop = nPop;
    this.remap = remap;
  }

  @Override
  public <T, F> Evolver<List<Double>, T, F> build(PrototypedFunctionBuilder<List<Double>, T> builder, T target, PartialComparator<F> comparator) {
    int length = builder.exampleFor(target).size();
    return new BasicEvolutionaryStrategy<>(
        builder.buildFor(target),
        new FixedLengthListFactory<>(length, new UniformDoubleFactory(-1d, 1d)),
        comparator.comparing(Individual::getFitness),
        sigma,
        nPop,
        nPop / 4,
        1,
        remap
    );
  }
}
