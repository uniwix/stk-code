#ifndef NET_NEURONS_HPP
#define NET_NEURONS_HPP

#include <vector>

#include "utils/random_generator.hpp"

namespace NetNeurons
{
	class Network
	{
	public:
		/**
		 * \brief Create an empty neuron network
		 */
		Network();

		/**
		 * \brief Create a neuron network with the given layers
		 * \param layers The number of neurons in each layer
		 */
		Network(const std::vector<int>& layers);

		/**
		 * \brief Compute the result of the network for the given inputs
		 * \param input Vector containing the inputs values
		 * \return The processed inputs by the network
		 */
		std::vector<double> compute(std::vector<double> input) const;

		/**
		 * \brief Constructor that creates a Network from an existing linking 
		 * \param data the input network
		 */
		Network(const std::vector<std::vector<std::vector<double>>>& data);


		/**
		 * \brief creates a copy of the network
		 * \return the current network
		 */
		std::vector<std::vector<std::vector<double>>> getNetwork();

	private:
		/**
		 * \brief Set the weights of a layer
		 * \param weights Vector containing the weights
		 */
		static void set_weights(std::vector<double>& weights);

		/**
		 * \brief Compute the result of a single layer of the network for the given inputs
		 * \param inputs Vector containing the inputs values
		 * \param layer Layer to compute
		 * \return The processed inputs by the layer
		 */
		static std::vector<double> compute_one_layer(const std::vector<double>& inputs, const std::vector<std::vector<double>>& layer);

		/**
		 * \brief Vector containing the layers of the network
		 */
		std::vector<std::vector<std::vector<double>>> m_layers_;
	};


}

#endif // NET_NEURONS_HPP
