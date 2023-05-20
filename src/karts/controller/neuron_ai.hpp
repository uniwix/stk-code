
//
//  SuperTuxKart - a fun racing game with go-kart
//  Copyright (C) 2004-2015  Steve Baker <sjbaker1@airmail.net>
//  Copyright (C) 2006-2015  Eduardo Hernandez Munoz
//  Copyright (C) 2010-2015  Joerg Henrichs
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public License
//  as published by the Free Software Foundation; either version 3
//  of the License, or (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

#ifndef HEADER_NEURON_AI_HPP
#define HEADER_NEURON_AI_HPP

// Some debugging features for the AI.

#ifdef DEBUG
   // Enable AI graphical debugging
#define AI_DEBUG
   // Enable AI raycast lines drawing
#define AI_DEBUG_RAYCAST
#endif


#include "karts/controller/ai_base_lap_controller.hpp"
#include "race/race_manager.hpp"
#include "tracks/drive_node.hpp"

#include <Network.hpp>
#include "graphics/show_curve.hpp"

class ItemManager;
class ItemState;
class LinearWorld;
class Track;

namespace irr
{
    namespace scene
    {
        class ISceneNode;
    }
}

/**
\brief AI class for neuron networks
\ingroup controller
*/
class NeuronAI : public AIBaseLapController
{
private:
    /*General purpose variables*/

    /** If set an item that the AI should aim for. */
    const ItemState *m_item_to_collect{};

    /** Score of the AI, used to determine the best AI */
    float m_score;

    /** The last item selected for collection, for which a probability
     *  was determined. */
    const ItemState* m_last_item_random{};

    ItemManager* m_item_manager;

#ifdef AI_DEBUG
    ShowCurve **m_curve;
#endif

    /**
     * \brief The neuron network associated with the AI
     */
    NetNeurons::Network m_neuron_network;

    float distanceToSide(float angle, const Vec3& pos, int curve=-1) const;
    float getDeltaScore(float dt, double dist_sum) const;

#ifdef AI_DEBUG_RAYCAST
    void drawRayCast(int curve, const Vec3& pos) const;
#endif

protected:
    unsigned int getNextSector(unsigned int index) override;

public:
                 NeuronAI(AbstractKart *kart);
                ~NeuronAI() override;
    void update      (int ticks) override;
    void reset       () override;
    virtual const irr::core::stringw& getNamePostfix() const;
    float getScore() const override { return m_score; }
};

#endif

/* EOF */
