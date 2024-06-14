//
//  SuperTuxKart - a fun racing game with go-kart
//  Copyright (C) 2004-2015 Steve Baker <sjbaker1@airmail.net>
//  Copyright (C) 2006-2015 Eduardo Hernandez Munoz
//  Copyright (C) 2008-2015 Joerg Henrichs
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


#include "karts/controller/neuron_ai.hpp"

#ifdef AI_DEBUG
#  include "graphics/irr_driver.hpp"
#endif
#include <fstream>

#include "graphics/show_curve.hpp"
#include "items/item_manager.hpp"
#include "karts/abstract_kart.hpp"
#include "karts/controller/kart_control.hpp"
#include "karts/kart_properties.hpp"
#include "karts/rescue_animation.hpp"
#include "modes/linear_world.hpp"
#include "race/race_manager.hpp"
#include "tracks/drive_graph.hpp"
#include "tracks/track.hpp"
#include "utils/constants.hpp"

#include "tracks/track_sector.hpp"
#include <sstream>

#ifdef AI_DEBUG
#  include "irrlicht.h"
   using namespace irr;
#endif

#include <cmath>
#include <iostream>

/*
static int callback(void* NotUsed, int argc, char** argv, char** azColName) {
	for (int i = 0; i < 11; ++i) {
        for (int j = 0; j < 6; ++j) {
            for (int k = 0; k < argc; ++k) {
                if (std::string(azColName[k]) == "coef_0_" + std::to_string(i) + "_" + std::to_string(j)) {
					gloal_network[0][i][j] = std::stof(argv[k]);
                }
			}
		}
	}
    for (int i = 0; i < 6; ++i) {
		for (int j = 0; j < 3; ++j) {
			for (int k = 0; k < argc; ++k) {
				if (std::string(azColName[k]) == "coef_1_" + std::to_string(i) + "_" + std::to_string(j)) {
					gloal_network[1][i][j] = std::stof(argv[k]);
				}
			}
		}
	}
	return 0;
}*/


NeuronAI::NeuronAI(AbstractKart *kart)
                   : AIBaseLapController(kart)
{
    m_item_manager = Track::getCurrentTrack()->getItemManager();
    NeuronAI::reset();
    m_score = 0.f;

    AIBaseController::setControllerName("NeuronNetwork");

    // Draw raycast lines
#ifdef AI_DEBUG
    
#define CURVE_RAYCAST_PI_2L 0
#define CURVE_RAYCAST_PI_3L 1
#define CURVE_RAYCAST_PI_6L 2
#define CURVE_RAYCAST_PI_2R 3
#define CURVE_RAYCAST_PI_3R 4
#define CURVE_RAYCAST_PI_6R 5
#define CURVE_RAYCAST_FRONT 6
#define NUM_CURVES (CURVE_RAYCAST_FRONT+1)
    if (!GUIEngine::isNoGraphics())
    {
        m_curve = new ShowCurve * [NUM_CURVES];
        for (unsigned int i = 0; i < NUM_CURVES; i++)
            m_curve[i] = nullptr;

#ifdef AI_DEBUG_RAYCAST
        m_curve[CURVE_RAYCAST_PI_2L] = new ShowCurve(0.1f, 0.1f,
            video::SColor(50, 128, 0, 0));
        m_curve[CURVE_RAYCAST_PI_3L] = new ShowCurve(0.1f, 0.1f,
            video::SColor(50, 128, 0, 0));
        m_curve[CURVE_RAYCAST_PI_6L] = new ShowCurve(0.1f, 0.1f,
            video::SColor(50, 128, 0, 0));
        m_curve[CURVE_RAYCAST_PI_2R] = new ShowCurve(0.1f, 0.1f,
            video::SColor(50, 0, 128, 0));
        m_curve[CURVE_RAYCAST_PI_3R] = new ShowCurve(0.1f, 0.1f,
            video::SColor(50, 0, 128, 0));
        m_curve[CURVE_RAYCAST_PI_6R] = new ShowCurve(0.1f, 0.1f,
            video::SColor(50, 0, 128, 0));
        m_curve[CURVE_RAYCAST_FRONT] = new ShowCurve(0.1f, 0.1f,
            video::SColor(50, 0, 0, 128));
#endif
    }
#else
#define CURVE_RAYCAST_PI_2L (-1)
#define CURVE_RAYCAST_PI_3L (-1)
#define CURVE_RAYCAST_PI_6L (-1)
#define CURVE_RAYCAST_PI_2R (-1)
#define CURVE_RAYCAST_PI_3R (-1)
#define CURVE_RAYCAST_PI_6R (-1)
#define CURVE_RAYCAST_FRONT (-1)
#endif

    if (!RaceManager::get()->getNeuronNetworkFile().empty())
    {
        try
        {
            Log::info("NeuronAI", ("Using Network: " + RaceManager::get()->getNeuronNetworkFile()).c_str());
            Log::info("NeuronAI", ("Using Session: " + std::to_string(RaceManager::get()->getSession())).c_str());

            // connexion au serveur MySQL
            auto my_sql = toml::parse("C:\\Users\\jbeno\\source\\repos\\uniwix\\genetic\\GeneticC\\mysql.toml");
            std::string host = toml::find<std::string>(my_sql, "host", "host");
            int port = toml::find<int>(my_sql, "host", "port");
            std::string user = toml::find<std::string>(my_sql, "client", "user");
            std::string password = toml::find<std::string>(my_sql, "client", "password");
            std::string database = toml::find<std::string>(my_sql, "client", "database");
            
            sql::Driver* driver = sql::mysql::get_driver_instance();
            sql::Connection* con(driver->connect("tcp://" + host + ":" + std::to_string(port), user, password));
            con->setSchema(database);  // selection de la base de donnees
            sql::PreparedStatement* pstmt = con->prepareStatement("SELECT reseau_bin, rec FROM Individu WHERE CONCAT(\"GEN\", generation, \"IND\", individu) = ? AND session = ?;");  // creation d'un objet pour executer des requetes sql
            
            pstmt->setString(1, RaceManager::get()->getNeuronNetworkFile());
            pstmt->setInt(2, RaceManager::get()->getSession());
            
            sql::ResultSet* res = pstmt->executeQuery();  // execution de la requete sql
            
            if (res->next())
            {
                std::istream* blob = res->getBlob("reseau_bin");
                std::istreambuf_iterator<char> isb = std::istreambuf_iterator<char>(*blob);
                std::vector<std::vector<std::vector<float>>> network = NeuralNetwork::deserialize({ isb, std::istreambuf_iterator<char>() });
                
                int rec = res->getInt("rec");
                std::vector<bool> rec_vector = NeuralNetwork::decode(rec, network.size() + 1);
	            
                m_neuron_network = NeuralNetwork::Network(network, rec_vector);
            }
            else
            {
                Log::fatal("NeuronAI", "No data found");
            }
            delete res;
            delete pstmt;
            delete con;
        }
        catch (sql::SQLException e)
        {
            Log::fatal("NeuronAI", "SQL error: %s", e.what());
        }
        /*
        int rc;
        char *zErrMsg = 0;
        sqlite3 *db;
        //std::ifstream file(R"(C:\Users\jbeno\source\repos\uniwix\AI\vec\)" + RaceManager::get()->getNeuronNetworkFile() + ".txt");
        rc = sqlite3_open("C:\\Users\\jbeno\\source\\repos\\uniwix\\genetic\\genetic.sqlite", &db);
        if (rc) {
            Log::fatal("NeuronAI", "Can't open database: %s", sqlite3_errmsg(db));
            exit(0);
        }
        else {
            Log::info("NeuronAI", "Opened database successfully");
        }
        std::stringstream sql;
        sqlite3_exec(db, sql.str().c_str(), callback, 0, &zErrMsg);
        if (rc != SQLITE_OK) {
			Log::fatal("NeuronAI", "SQL error: %s", zErrMsg);
			sqlite3_free(zErrMsg);
            exit(0);
		}
		else {
			Log::verbose("NeuronAI", "Data imported");
		}
        sqlite3_close(db);*/
        /*assert(m_neuron_network.get_layers().size() == 2);
        assert(m_neuron_network.get_layers()[0].size() == 6);
        assert(m_neuron_network.get_layers()[0][0].size() == 11);
        assert(m_neuron_network.get_layers()[1].size() == 3);
        assert(m_neuron_network.get_layers()[1][0].size() == 6);*/
    }
    else
    {

        Log::info("NeuronAI", ("Using Network file not working: " + RaceManager::get()->getNeuronNetworkFile()).c_str());
        m_neuron_network = NeuralNetwork::Network(std::vector<int>({ 11, 6, 3 }));
    }
}   // NeuronAI

//-----------------------------------------------------------------------------
/** Destructor, mostly to clean up debug data structures.
 */
NeuronAI::~NeuronAI()
{
#ifdef AI_DEBUG
    if (!GUIEngine::isNoGraphics())
    {
        for (unsigned int i = 0; i < NUM_CURVES; i++)
        {
            delete m_curve[i];
        }
        delete[] m_curve;
    }
#endif
}   // ~NeuronAI

//-----------------------------------------------------------------------------
/** Resets the AI when a race is restarted.
 */
void NeuronAI::reset()
{
    m_item_to_collect            = nullptr;
    m_last_item_random           = nullptr;
    m_score                      = 0.f;

    AIBaseLapController::reset();
    m_track_node = Graph::UNKNOWN_SECTOR;
    DriveGraph::get()->findRoadSector(m_kart->getXYZ(), &m_track_node);
    if (m_track_node == Graph::UNKNOWN_SECTOR)
    {
        Log::error(getControllerName().c_str(),
            "Invalid starting position for '%s' - not on track"
            " - can be ignored.",
            m_kart->getIdent().c_str());
        m_track_node = DriveGraph::get()->findOutOfRoadSector(m_kart->getXYZ());
    }

    AIBaseLapController::reset();
}   // reset

//-----------------------------------------------------------------------------
/** Returns a name for the AI.
 *  This is used in profile mode when comparing different AI implementations
 *  to be able to distinguish them from each other.
 */
const irr::core::stringw& NeuronAI::getNamePostfix() const
{
    // Static to avoid returning the address of a temporary string.
    static irr::core::stringw name="(neuron network)";
    return name;
}   // getNamePostfix

//-----------------------------------------------------------------------------
/** Returns the pre-computed successor of a graph node.
 *  \param index The index of the graph node for which the successor
 *               is searched.
 */
unsigned int NeuronAI::getNextSector(const unsigned int index)
{
    return m_successor_index[index];
}   // getNextSector

//-----------------------------------------------------------------------------
/** This is the main entry point for the AI.
 *  It is called once per frame for each AI and determines the behaviour of
 *  the AI, e.g. steering, accelerating/braking, firing.
 */
void NeuronAI::update(const int ticks)
{
	const float dt = stk_config->ticks2Time(ticks);

    // Clear stored items if they were deleted (for example a switched nitro)
    if (m_item_to_collect &&
        !m_item_manager->itemExists(m_item_to_collect))
        m_item_to_collect = nullptr;
    if (m_last_item_random &&
        !m_item_manager->itemExists(m_last_item_random))
        m_last_item_random = nullptr;

    m_controls->setRescue(false);

    // This is used to enable firing an item backwards.
    m_controls->setLookBack(false);
    m_controls->setNitro(false);

    // Don't do anything if there is currently a kart animations shown.
    if(m_kart->getKartAnimation())
        return;

    if( m_world->isStartPhase() )
    {
        AIBaseLapController::update(ticks);
        return;
    }

    // Get distances with raycasts
	const auto d1 = static_cast<float>(distanceToSide(M_PI / 2.,  m_kart->getXYZ(), CURVE_RAYCAST_PI_2L));
	const auto d2 = static_cast<float>(distanceToSide(M_PI / 3.,  m_kart->getXYZ(), CURVE_RAYCAST_PI_3L));
	const auto d3 = static_cast<float>(distanceToSide(M_PI / 6.,  m_kart->getXYZ(), CURVE_RAYCAST_PI_6L));
	const auto d4 = static_cast<float>(distanceToSide(0.,         m_kart->getXYZ(), CURVE_RAYCAST_FRONT));
	const auto d5 = static_cast<float>(distanceToSide(M_PI / -6., m_kart->getXYZ(), CURVE_RAYCAST_PI_6R));
	const auto d6 = static_cast<float>(distanceToSide(M_PI / -3., m_kart->getXYZ(), CURVE_RAYCAST_PI_3R));
	const auto d7 = static_cast<float>(distanceToSide(M_PI / -2., m_kart->getXYZ(), CURVE_RAYCAST_PI_2R));

    // Get the inputs for the neural network
    const std::vector<float> inputs({
        d1,
        d2,
        d3,
    	d4,
    	d5,
    	d6,
    	d7,
        static_cast<float>(getAngle()),
        static_cast<float>(distanceToCenter()),
        static_cast<float>(m_kart->getSpeed()),
        static_cast<float>(m_kart->getSteerPercent()),
    });

    // Compute the outputs of the neural network
    const std::vector<float> outs = m_neuron_network.compute(inputs);
    
    // Get the outputs in the right format
    const auto steer(static_cast<float>(outs[0]));
	const auto acc(static_cast<float>(outs[1]));
    const bool brake = outs[2] < 0.;

    // Set the controls of the kart according to the outputs of the neural network
    m_controls->setSteer(steer);
    m_controls->setAccel(acc);
    m_controls->setBrake(brake);

    // Update the score
    m_score += getDeltaScore(dt, d1+d2+d3+d4+d5+d6+d7);
    
    // Show score
	const irr::core::stringw str_score = std::to_string(static_cast<int>(m_score)).c_str();
    m_kart->setOnScreenText(str_score);

    /*And obviously general kart stuff*/
    AIBaseLapController::update(ticks);
}   // update

float NeuronAI::getAngle()
{
    // check if the player is going in the wrong direction
    const DriveNode* node = DriveGraph::get()->getNode(m_track_node);
    Vec3 center_line = node->getUpperCenter() - node->getLowerCenter();
    float angle_diff = m_kart->getVelocity().angle(center_line);

    if (angle_diff > M_PI)
        angle_diff -= 2 * M_PI;
    else if (angle_diff < -M_PI)
        angle_diff += 2 * M_PI;
    return angle_diff / M_PI;
}

float NeuronAI::distanceToCenter()
{
    return m_world->getDistanceToCenterForKart(m_kart->getWorldKartId());
}

//-----------------------------------------------------------------------------
/**
 * \brief Get the score gain during the time dt.
 * \param dt The time during which the score gain is computed.
 * \param dist_sum The sum of the distances to the road side. Used to penalize AI that are out of the road.
 * \return  The score gain
 */
float NeuronAI::getDeltaScore(const float dt, const float dist_sum) const
{
    const int sector = m_world->getTrackSector(m_kart->getWorldKartId())->getCurrentGraphNode();

    if (DriveGraph::get()->getNumberOfSuccessors(sector) > 1)
        return 0;

    const DriveNode* node = DriveGraph::get()->getNode(sector);
    const Vec3 center_line = node->getUpperCenter() - node->getLowerCenter();
    float dscore = m_kart->getVelocity().dot(center_line.normalized()) * dt;

    if (dist_sum < 1.)
    {
	    dscore -= m_kart->getVelocity().normalized().dot(m_kart->getVelocity()) * dt;
    }
    return dscore;
}  // getDeltaScore

/**
 * \brief Compute the distance to the road side following the given angle
 * \param angle angle of distance check from ahead
 * \param pos position of the kart
 * \param curve the curve index to use to draw the raycast
 * \return the distance to the road side
 */
float NeuronAI::distanceToSide(const float angle, const Vec3& pos, const int curve) const
{
	constexpr int steps = 1000;
    int d_node = m_track_node;
	const Vec3 dir_vec = Vec3(0.0f, 0.f, 1.f).rotate(Vec3(0.f, 1.f, 0.f), angle).rotate(m_kart->getRotation().getAxis(), m_kart->getRotation().getAngle());
    for (int d = 0; steps > d; ++d)
    {
        Vec3 step_coord = pos + dir_vec * static_cast<float>(d);

        if (d_node != Graph::UNKNOWN_SECTOR &&
            m_next_node_index[d_node] != -1)
            DriveGraph::get()->findRoadSector(step_coord, &d_node);

        if (d_node == Graph::UNKNOWN_SECTOR)
        {
#ifdef AI_DEBUG_RAYCAST
            if (curve >= 0 && !GUIEngine::isNoGraphics())
                drawRayCast(curve, step_coord);
#endif
            return static_cast<float>(d);
        }
    }
#ifdef AI_DEBUG_RAYCAST
    if (!GUIEngine::isNoGraphics())
    {
        Vec3 step_coord = pos + steps * dir_vec;
        if (curve >= 0)
            drawRayCast(curve, step_coord);
    }
#endif
    return static_cast<float>(steps);
}   // distanceToSide

#ifdef AI_DEBUG_RAYCAST
void NeuronAI::drawRayCast(const int curve, const Vec3 &pos) const
{
    m_curve[curve]->clear();
    m_curve[curve]->addPoint(m_kart->getXYZ());
    m_curve[curve]->addPoint(pos);
}  // drawRayCast
#endif
