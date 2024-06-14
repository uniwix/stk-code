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


#include "karts/controller/qlearning_ai.hpp"

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

QLearningAI::QLearningAI(AbstractKart *kart)
                   : AIBaseLapController(kart)
{
    m_item_manager = Track::getCurrentTrack()->getItemManager();
    QLearningAI::reset();
    m_score = 0.f;

    AIBaseController::setControllerName("QLearning");

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
			if (RaceManager::get()->isTraining())
                Log::info("QLearningAI", "Training stored Network");
            else
                Log::info("QLearningAI", "Using stored Network");

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
            sql::PreparedStatement* pstmt = con->prepareStatement("SELECT reseau_bin, rec FROM Individu WHERE session = ? and individu = ?;");  // creation d'un objet pour executer des requetes sql
            
            pstmt->setInt(1, RaceManager::get()->getSession());
            pstmt->setInt(2, std::stoi(RaceManager::get()->getNeuronNetworkFile()));
            
            sql::ResultSet* res = pstmt->executeQuery();  // execution de la requete sql
            
            if (res->next())
            {
                std::istream* blob = res->getBlob("reseau_bin");
                std::istreambuf_iterator<char> isb = std::istreambuf_iterator<char>(*blob);
                std::vector<std::vector<std::vector<float>>> network = NeuralNetwork::deserialize({ isb, std::istreambuf_iterator<char>() });
                
                int rec = res->getInt("rec");
                std::vector<bool> rec_vector = NeuralNetwork::decode(rec, network.size() + 1);
	            
                m_neuron_network = NeuralNetwork::Network(network, rec_vector);

                Log::info("QLearningAI", "Network loaded");
            }
            else
            {
                Log::fatal("QLearningAI", "No data found");
            }
            delete res;
            delete pstmt;
            delete con;
        }
        catch (sql::SQLException e)
        {
            Log::fatal("QLearningAI", "SQL error: %s", e.what());
        }
    }
    else
    {

        Log::info("QLearningAI", ("Using Network file not working: " + RaceManager::get()->getNeuronNetworkFile()).c_str());
        m_neuron_network = NeuralNetwork::Network(11, 6, 5, 3);

    }

    xt = std::vector<float>(11);
    qmax = 0.f;
    imax = 0;
    wires = std::vector<std::vector<float>>(m_neuron_network.getWireCount());
    q_values = std::vector<float>(m_neuron_network.getWireCount());

	Log::info("QLearningAI", "QLearningAI controller created");
	RaceManager::get()->setNetwork(&m_neuron_network);
}   // NeuronAI

//-----------------------------------------------------------------------------
/** Destructor, mostly to clean up debug data structures.
 */
QLearningAI::~QLearningAI()
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
void QLearningAI::reset()
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
const irr::core::stringw& QLearningAI::getNamePostfix() const
{
    // Static to avoid returning the address of a temporary string.
    static irr::core::stringw name="(q-learning)";
    return name;
}   // getNamePostfix

//-----------------------------------------------------------------------------
/** Returns the pre-computed successor of a graph node.
 *  \param index The index of the graph node for which the successor
 *               is searched.
 */
unsigned int QLearningAI::getNextSector(const unsigned int index)
{
    return m_successor_index[index];
}   // getNextSector

//-----------------------------------------------------------------------------
/** This is the main entry point for the AI.
 *  It is called once per frame for each AI and determines the behaviour of
 *  the AI, e.g. steering, accelerating/braking, firing.
 */
void QLearningAI::update(const int ticks)
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
    if (m_kart->getKartAnimation())
    {
        is_not_first = false;  // reset the learning
        return;
    }

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

    float delta_score = getDeltaScore(dt, d1+d2+d3+d4+d5+d6+d7) / 10.f;

	
    if ((RaceManager::get()->isTraining() || true) && is_not_first)
	{
        /*/ Utilisation des informations de la base de données pour l'apprentissage
        try
        {
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
            sql::PreparedStatement* pstmt = con->prepareStatement("SELECT delta_score, data, imax FROM q_learning WHERE id = ?");  // creation d'un objet pour executer des requetes sql
            int id = rand() % 1000 + 1;
            pstmt->setInt(1, id);

            sql::ResultSet* res = pstmt->executeQuery();  // execution de la requete sql
            if (res->next())
            {
                float d_score = res->getDouble("delta_score");

                if (d_score > -10)
                {
                    std::istream* blob = res->getBlob("data");
                    std::istreambuf_iterator<char> isb = std::istreambuf_iterator<char>(*blob);
                    std::vector<std::vector<std::vector<float>>> data = NeuralNetwork::deserialize({ isb, std::istreambuf_iterator<char>() });

                    int i_max = res->getInt("imax");

                    m_neuron_network.wire_fit(data[0][0], data[0][1], d_score, data[2], data[1][0], i_max, 0.1, 0.1, 0.0000001, 0.);  // TODO Change the parameters
                }

                if (!exit_next_time)
                {
                    std::vector<int8_t> serialized = NeuralNetwork::serialize({ { xt, inputs },  { q_values }, wires });
                    std::stringstream stream;
                    stream = std::stringstream(std::string(serialized.begin(), serialized.end()));

                    pstmt = con->prepareStatement("UPDATE q_learning SET data = ?, delta_score = ?, imax = ? WHERE id = ?;");  // creation d'un objet pour executer des requetes sql
                    pstmt->setBlob(1, &stream);
                    pstmt->setDouble(2, delta_score);
                    pstmt->setInt(3, imax);
                    pstmt->setInt(4, id);
                    pstmt->execute();
                }
            }
            else
            {
                Log::error("QLearningAI", "No data found");
            }

            /* Not nedeed because the table is full
            pstmt = con->prepareStatement("SELECT count(id) FROM q_learning;");  // creation d'un objet pour executer des requetes sql
            res = pstmt->executeQuery();  // execution de la requete sql
            if (res->next())
            {
                int count = res->getInt(1);
                if (count < 1000)
                {
                    Log::error("QLearningAI", "No minimum in database. Adding this position.");
                    std::vector<int8_t> data = Network::serialize({ { xt, inputs },  { q_values }, wires });
                    std::stringstream stream;
                    stream = std::stringstream(std::string(data.begin(), data.end()));

                    pstmt = con->prepareStatement("INSERT INTO q_learning (data, delta_score, imax) VALUES (?, ?, ?);");  // creation d'un objet pour executer des requetes sql
                    pstmt->setBlob(1, &stream);
                    pstmt->setDouble(2, delta_score);
                    pstmt->setInt(3, imax);
                    pstmt->execute();
                }
                else
                {
                    Log::error("QLearningAI", "Database is full. No new data added.");
                }
            }
            else
            {
                Log::error("QLearningAI", "No size returned while looking in q_learning table");
            }*//*

            delete res;
            delete pstmt;
            delete con;
        }
        catch (sql::SQLException e)
        {
            Log::fatal("QLearningAI", "SQL error: %s", e.what());
        }*/

		m_neuron_network.wire_fit(xt, inputs, delta_score, wires, q_values, imax, 1, 0.5, 0.0000001, 0);
        if (exit_next_time)
        {
            World::getWorld()->enterRaceOverState();
            return;
        }
	}
    else
    {
		is_not_first = true;
    }

    // Compute the outputs of the neural network
    const std::vector<float> sorties = m_neuron_network.compute(inputs);
 
	for (size_t i = 0; i < m_neuron_network.getWireCount(); ++i)
    {
        size_t s_index = i * (m_neuron_network.getControlsCount() + 1);
        q_values[i] = sorties[s_index];
        wires[i] = std::vector<float>( sorties.begin() + s_index + 1, sorties.begin() + s_index + m_neuron_network.getControlsCount() + 1 );
	}


    qmax = 0.f;
    imax = 0;

    for (int i = 0; i < m_neuron_network.getWireCount(); ++i)
    {
        if (q_values[i] > qmax)
        {
            qmax = q_values[i];
            imax = i;
        }
    }
    
    // Get the outputs in the right format
    const auto steer(static_cast<float>(wires[imax][0]));
	const auto acc(static_cast<float>(wires[imax][1]));
    const bool brake = wires[imax][2] < 0.;

    // Set the controls of the kart according to the outputs of the neural network
    m_controls->setSteer(steer);
    m_controls->setAccel(acc);
    m_controls->setBrake(brake);

    // Update the score
	m_score += delta_score * dt * 10;
    
    // Show score
	const irr::core::stringw str_score = std::to_string(static_cast<int>(m_score)).c_str();
    m_kart->setOnScreenText(str_score);

    /*And obviously general kart stuff*/
    AIBaseLapController::update(ticks);
}   // update

float QLearningAI::getAngle()
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

float QLearningAI::distanceToCenter()
{
    return m_world->getDistanceToCenterForKart(m_kart->getWorldKartId());
}

float min(float a, float b)
{
	return a < b ? a : b;
}

//-----------------------------------------------------------------------------
/**
 * \brief Get the score gain during the time dt.
 * \param dt The time during which the score gain is computed.
 * \param dist_sum The sum of the distances to the road side. Used to penalize AI that are out of the road.
 * \return  The score gain
 */
float QLearningAI::getDeltaScore(const float dt, const float dist_sum)
{
    const int sector = m_world->getTrackSector(m_kart->getWorldKartId())->getCurrentGraphNode();

    if (DriveGraph::get()->getNumberOfSuccessors(sector) > 1)
        return 0;

    const DriveNode* node = DriveGraph::get()->getNode(sector);
    const Vec3 center_line = node->getUpperCenter() - node->getLowerCenter();
	if (m_kart->getSpeed() < 0.01)
		return 0;
    float dscore = m_kart->getVelocity().normalized().dot(center_line.normalized()) * min(m_kart->getKartProperties()->getEngineMaxSpeed(), m_kart->getSpeed());

    if (dist_sum < 1.)
    {
        exit_next_time = true;
        dscore = -20.f;  //-= m_kart->getSpeed();
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
float QLearningAI::distanceToSide(const float angle, const Vec3& pos, const int curve) const
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
void QLearningAI::drawRayCast(const int curve, const Vec3 &pos) const
{
    m_curve[curve]->clear();
    m_curve[curve]->addPoint(m_kart->getXYZ());
    m_curve[curve]->addPoint(pos);
}  // drawRayCast
#endif
