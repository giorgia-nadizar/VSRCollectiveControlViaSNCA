package it.units.erallab.evolution.builder.evolver;

import it.units.erallab.evolution.builder.PrototypedFunctionBuilder;
import it.units.malelab.jgea.core.IndependentFactory;
import it.units.malelab.jgea.core.Individual;
import it.units.malelab.jgea.core.evolver.Evolver;
import it.units.malelab.jgea.core.evolver.StandardEvolver;
import it.units.malelab.jgea.core.evolver.StandardWithEnforcedDiversityEvolver;
import it.units.malelab.jgea.core.operator.Mutation;
import it.units.malelab.jgea.core.order.PartialComparator;
import it.units.malelab.jgea.core.selector.Last;
import it.units.malelab.jgea.core.selector.Tournament;
import it.units.malelab.jgea.representation.sequence.FixedLengthListFactory;
import it.units.malelab.jgea.representation.sequence.ProbabilisticMutation;
import it.units.malelab.jgea.representation.sequence.UniformCrossover;

import java.util.List;
import java.util.Map;

/**
 * @author eric
 */
public class IntegersStandard implements EvolverBuilder<List<Integer>> {

  private final int nPop;
  private final int nTournament;
  private final double xOverProb;
  private final boolean diversityEnforcement;
  private final boolean remap;

  public IntegersStandard(int nPop, int nTournament, double xOverProb, boolean diversityEnforcement, boolean remap) {
    this.nPop = nPop;
    this.nTournament = nTournament;
    this.xOverProb = xOverProb;
    this.diversityEnforcement = diversityEnforcement;
    this.remap = remap;
  }

  @Override
  public <T, F> Evolver<List<Integer>, T, F> build(PrototypedFunctionBuilder<List<Integer>, T> builder, T target, PartialComparator<F> comparator) {
    int length = builder.exampleFor(target).size();
    int maxValue = builder.exampleFor(target).get(0);
    IndependentFactory<List<Integer>> factory = new FixedLengthListFactory<>(length, random -> random.nextInt(maxValue));
    Mutation<Integer> mutation = (i, random) -> random.nextInt(maxValue);
    double pMut = Math.max(0.01, 1d / (double) length);
    if (!diversityEnforcement) {
      return new StandardEvolver<>(
          builder.buildFor(target),
          factory,
          comparator.comparing(Individual::getFitness),
          nPop,
          Map.of(
              new ProbabilisticMutation<>(pMut, factory, mutation), 1d - xOverProb,
              new UniformCrossover<>(factory)
                  .andThen(new ProbabilisticMutation<>(pMut, factory, mutation)), xOverProb
          ),
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
        Map.of(
            new ProbabilisticMutation<>(1d / (double) length, factory, mutation), 1d - xOverProb,
            new UniformCrossover<>(factory), xOverProb
        ),
        new Tournament(nTournament),
        new Last(),
        nPop,
        true,
        remap,
        100
    );
  }

}
