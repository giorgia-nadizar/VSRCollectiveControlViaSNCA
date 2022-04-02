package it.units.erallab.evolution.builder.evolver;

import com.google.common.collect.Range;
import it.units.erallab.evolution.builder.PrototypedFunctionBuilder;
import it.units.malelab.jgea.core.Factory;
import it.units.malelab.jgea.core.Individual;
import it.units.malelab.jgea.core.evolver.Evolver;
import it.units.malelab.jgea.core.evolver.StandardEvolver;
import it.units.malelab.jgea.core.evolver.StandardWithEnforcedDiversityEvolver;
import it.units.malelab.jgea.core.operator.Crossover;
import it.units.malelab.jgea.core.operator.GeneticOperator;
import it.units.malelab.jgea.core.operator.Mutation;
import it.units.malelab.jgea.core.order.PartialComparator;
import it.units.malelab.jgea.core.selector.Last;
import it.units.malelab.jgea.core.selector.Tournament;
import it.units.malelab.jgea.core.util.Pair;
import it.units.malelab.jgea.representation.sequence.FixedLengthListFactory;
import it.units.malelab.jgea.representation.sequence.UniformCrossover;
import it.units.malelab.jgea.representation.sequence.bit.BitFlipMutation;
import it.units.malelab.jgea.representation.sequence.bit.BitString;
import it.units.malelab.jgea.representation.sequence.bit.BitStringFactory;
import it.units.malelab.jgea.representation.sequence.numeric.GaussianMutation;
import it.units.malelab.jgea.representation.sequence.numeric.GeometricCrossover;
import it.units.malelab.jgea.representation.sequence.numeric.UniformDoubleFactory;

import java.util.List;
import java.util.Map;

/**
 * @author eric
 */
public class BinaryAndDoublesStandard implements EvolverBuilder<Pair<BitString, List<Double>>> {

  private final int nPop;
  private final int nTournament;
  private final double xOverProb;
  private final boolean diversityEnforcement;
  private final boolean remap;

  public BinaryAndDoublesStandard(int nPop, int nTournament, double xOverProb, boolean diversityEnforcement, boolean remap) {
    this.nPop = nPop;
    this.nTournament = nTournament;
    this.xOverProb = xOverProb;
    this.diversityEnforcement = diversityEnforcement;
    this.remap = remap;
  }

  @Override
  public <T, F> Evolver<Pair<BitString, List<Double>>, T, F> build(PrototypedFunctionBuilder<Pair<BitString, List<Double>>, T> builder, T target, PartialComparator<F> comparator) {
    Pair<BitString, List<Double>> sampleGenotype = builder.exampleFor(target);
    int bitStringLength = sampleGenotype.first().size();
    int doublesLength = sampleGenotype.second().size();
    Factory<Pair<BitString, List<Double>>> factory = Factory.pair(
        new BitStringFactory(bitStringLength),
        new FixedLengthListFactory<>(doublesLength, new UniformDoubleFactory(-1d, 1d)));
    Map<GeneticOperator<Pair<BitString, List<Double>>>, Double> geneticOperators = Map.of(
        Mutation.pair(
            new BitFlipMutation(.01d),
            new GaussianMutation(.35d)
        ), 1d - xOverProb,
        Crossover.pair(
            new UniformCrossover<>(new BitStringFactory(bitStringLength)),
            new GeometricCrossover(Range.closed(-.5d, 1.5d))
        ).andThen(
            Mutation.pair(new BitFlipMutation(.01d), new GaussianMutation(.1d))
        ), xOverProb
    );
    if (!diversityEnforcement) {
      return new StandardEvolver<>(
          builder.buildFor(target),
          factory,
          comparator.comparing(Individual::getFitness),
          nPop,
          geneticOperators,
          new Tournament(nTournament),
          new Last(),
          nPop,
          true,
          remap
      );
    }
    return new StandardWithEnforcedDiversityEvolver<>(
        builder.buildFor(target),
        factory,
        comparator.comparing(Individual::getFitness),
        nPop,
        geneticOperators,
        new Tournament(nTournament),
        new Last(),
        nPop,
        true,
        remap,
        100
    );
  }

}
