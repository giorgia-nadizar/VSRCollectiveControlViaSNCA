package it.units.erallab.evolution.builder.evolver;

import it.units.erallab.evolution.builder.PrototypedFunctionBuilder;
import it.units.malelab.jgea.core.evolver.Evolver;
import it.units.malelab.jgea.core.order.PartialComparator;

/**
 * @author eric
 */
public interface EvolverBuilder<G> {
  <T, F> Evolver<G, T, F> build(PrototypedFunctionBuilder<G, T> builder, T target, PartialComparator<F> comparator);
}
