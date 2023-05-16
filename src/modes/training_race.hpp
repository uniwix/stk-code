//  SuperTuxKart - a fun racing game with go-kart
//  Copyright (C) 2004-2015 SuperTuxKart-Team
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

#ifndef _training_race_
#define _training_race_

#include "modes/linear_world.hpp"

/**
  * \brief Represents a training race, where AI are trained
  * \ingroup modes
  */
class TrainingRace : public LinearWorld
{
protected:
    // clock events
    virtual bool  isRaceOver() OVERRIDE;

public:
                  TrainingRace();
    virtual      ~TrainingRace() {};

    // overriding World methods
    virtual void  getDefaultCollectibles(int *collectible_type,
                                         int *amount) OVERRIDE;
    virtual bool  haveBonusBoxes() OVERRIDE;
    virtual const std::string& getIdent() const OVERRIDE { return IDENT_NAI;  }

    virtual void  endRaceEarly() OVERRIDE;
};

#endif
