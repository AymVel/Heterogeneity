#include "flamegpu/flamegpu.h"
#include <cmath>

FLAMEGPU_AGENT_FUNCTION(comm_social, flamegpu::MessageSpatial2D, flamegpu::MessageSpatial2D) {
    float radius = 0.5f;

    float x = FLAMEGPU->getVariable<float>("x");
    float y = FLAMEGPU->getVariable<float>("y");
    const flamegpu::id_t ID = FLAMEGPU->getID();
    // Get previous velocity
    float prev_vx = FLAMEGPU->getVariable<float>("vx");
    float prev_vy = FLAMEGPU->getVariable<float>("vy");

    int count = 0;
    float sumNeighbors = 0; // Initialize sum of neighbors
    for (const auto& message : FLAMEGPU->message_in(x, y)) {
        if (message.getVariable<int>("type") == 1) {
            const float x2 = message.getVariable <float > ("x");
           const float y2 = message.getVariable <float > ("y");

           float x21 = x2 - x;
           float y21 = y2 - y;
           const float separation = sqrt(x21 * x21 + y21 * y21);
            if (separation < radius) {
                const int neighbors = message.getVariable<int>("neighbors");
                sumNeighbors += neighbors; // Add neighbors to sum
                count++; // Increment count of neighbors
            }
        }
    }
    float meanNeighbors = 0.0f;
    if (count > 0) {
        meanNeighbors = sumNeighbors / count; // Calculate mean if count is greater than zero
    }

    // Add persistence factor
    const float persistence = 0.7f;

    // Add some random perturbation to the previous velocity
    float noise_angle = FLAMEGPU->random.uniform<float>() * 3.14159f * 2.0f;
    float noise_vx = cos(noise_angle);
    float noise_vy = sin(noise_angle);

    // Combine persistent velocity and noise
    float vx = persistence * prev_vx + (1 - persistence) * noise_vx;
    float vy = persistence * prev_vy + (1 - persistence) * noise_vy;

    // Normalize the velocity vector
    float length = sqrt(vx * vx + vy * vy);
    vx /= length;
    vy /= length;



     float max = ((radius*2)/0.06f)*((radius*2)/0.06f)*0.75f;
    float step_size =0.15f-meanNeighbors/max;
    if (step_size <0){
        step_size=0;
    }
    x += step_size * vx;
    y += step_size * vy;

    // Ensure the new position is within the environment bounds
    float width = FLAMEGPU->environment.getProperty<float>("ENV_WIDTH");
    // Wrap around if agents move outside the environment bounds
    if (x < 0)
        x += width;
    else if (x >= width)
        x -= width;
    if (y < 0)
        y += width;
    else if (y >= width)
        y -= width;

    // Update agent positions and velocities
    FLAMEGPU->setVariable<float>("x", x);
    FLAMEGPU->setVariable<float>("y", y);
    FLAMEGPU->setVariable<float>("vx", vx);
    FLAMEGPU->setVariable<float>("vy", vy);

    FLAMEGPU -> message_out.setVariable < int > ("id", FLAMEGPU ->getID());
    FLAMEGPU -> message_out.setVariable < int > ("type", FLAMEGPU ->getVariable <int > ("type"));
    FLAMEGPU -> message_out.setLocation(FLAMEGPU -> getVariable <float > ("x"),FLAMEGPU -> getVariable <float > ("y"));

    return flamegpu::ALIVE;
}

FLAMEGPU_AGENT_FUNCTION(comm_informed, flamegpu::MessageSpatial2D, flamegpu::MessageNone) {
    float radius = 0.5f;

    float x = FLAMEGPU->getVariable<float>("x");
    float y = FLAMEGPU->getVariable<float>("y");
    const flamegpu::id_t ID = FLAMEGPU->getID();
    // Get previous velocity
    float prev_vx = FLAMEGPU->getVariable<float>("vx");
    float prev_vy = FLAMEGPU->getVariable<float>("vy");

    int count = 0;
    for (const auto & message: FLAMEGPU ->message_in(x, y)) {
      if (message.getVariable <flamegpu::id_t > ("id") != ID) {

       const float x2 = message.getVariable <float > ("x");
       const float y2 = message.getVariable <float > ("y");

       float x21 = x2 - x;
       float y21 = y2 - y;
       const float separation = sqrt(x21 * x21 + y21 * y21);
        if (separation < radius) {
          count++;
        }
      }
    }



    // Add persistence factor
    const float persistence = 0.7f;

    // Add some random perturbation to the previous velocity
    float noise_angle = FLAMEGPU->random.uniform<float>() * 3.14159f * 2.0f;
    float noise_vx = cos(noise_angle);
    float noise_vy = sin(noise_angle);

    // Combine persistent velocity and noise
    float vx = persistence * prev_vx + (1 - persistence) * noise_vx;
    float vy = persistence * prev_vy + (1 - persistence) * noise_vy;

    // Normalize the velocity vector
    float length = sqrt(vx * vx + vy * vy);
    vx /= length;
    vy /= length;



     float max = ((radius*2)/0.06f)*((radius*2)/0.06f)*0.75f;
    float step_size =0.15f-count/max;
    if (step_size <0){
        step_size=0;
    }
    x += step_size * vx;
    y += step_size * vy;

    // Ensure the new position is within the environment bounds
    float width = FLAMEGPU->environment.getProperty<float>("ENV_WIDTH");
    // Wrap around if agents move outside the environment bounds
    if (x < 0)
        x += width;
    else if (x >= width)
        x -= width;
    if (y < 0)
        y += width;
    else if (y >= width)
        y -= width;

    // Update agent positions and velocities
    FLAMEGPU->setVariable<float>("x", x);
    FLAMEGPU->setVariable<float>("y", y);
    FLAMEGPU->setVariable<float>("vx", vx);
    FLAMEGPU->setVariable<float>("vy", vy);
    FLAMEGPU->setVariable<int>("neighbors", count);



    return flamegpu::ALIVE;
}

FLAMEGPU_AGENT_FUNCTION(output_message,flamegpu::MessageNone,flamegpu::MessageSpatial2D) {
    FLAMEGPU -> message_out.setVariable < int > ("id", FLAMEGPU ->getID());
    FLAMEGPU -> message_out.setVariable < int > ("neighbors", FLAMEGPU ->getVariable <int > ("neighbors"));
FLAMEGPU -> message_out.setVariable < int > ("type", FLAMEGPU ->getVariable <int > ("type"));
    FLAMEGPU -> message_out.setLocation(FLAMEGPU -> getVariable <float > ("x"),FLAMEGPU -> getVariable <float > ("y"));
    return flamegpu::ALIVE;
}
FLAMEGPU_AGENT_FUNCTION(output_message_social,flamegpu::MessageNone,flamegpu::MessageSpatial2D) {
    FLAMEGPU -> message_out.setVariable < int > ("id", FLAMEGPU ->getID());
    FLAMEGPU -> message_out.setVariable < int > ("neighbors", FLAMEGPU ->getVariable <int > ("neighbors"));
    FLAMEGPU -> message_out.setVariable < int > ("type", FLAMEGPU ->getVariable <int > ("type"));
    FLAMEGPU -> message_out.setLocation(FLAMEGPU -> getVariable <float > ("x"),FLAMEGPU -> getVariable <float > ("y"));
    return flamegpu::ALIVE;
}
FLAMEGPU_INIT_FUNCTION(create_self) {
        const unsigned int AGENT_COUNT = FLAMEGPU -> environment.getProperty < unsigned int > ("AGENT_COUNT");
        const float ENV_WIDTH = FLAMEGPU -> environment.getProperty <float > ("ENV_WIDTH");
        // Create agents
        flamegpu::HostAgentAPI t_pop = FLAMEGPU -> agent("self-informed");
        for (unsigned int i = 0; i < AGENT_COUNT/2; ++i) {
            auto t = t_pop.newAgent();
            t.setVariable<float>("x", FLAMEGPU->random.uniform<float>() * ENV_WIDTH);
            t.setVariable<float>("y", FLAMEGPU->random.uniform<float>() * ENV_WIDTH);
            t.setVariable<float>("vx", 0.0f);
            t.setVariable<float>("vy", 0.0f);
            t.setVariable<int>("neighbors", 0);
            t.setVariable<int>("type", 1);
        }
        flamegpu::HostAgentAPI s_pop = FLAMEGPU -> agent("social");
        for (unsigned int i = 0; i < AGENT_COUNT/2; ++i) {
            auto s = s_pop.newAgent();
            s.setVariable<float>("x", FLAMEGPU->random.uniform<float>() * ENV_WIDTH);
            s.setVariable<float>("y", FLAMEGPU->random.uniform<float>() * ENV_WIDTH);
            s.setVariable<float>("vx", 0.0f);
            s.setVariable<float>("vy", 0.0f);
            s.setVariable<int>("neighbors", 0);
            s.setVariable<int>("type", 0);
        }
}




int main(int argc, const char ** argv) {
    // Define some useful constants
    const float ENV_WIDTH = 20;
    const unsigned int AGENT_COUNT = 1000*ENV_WIDTH;


    // Define the FLAME GPU model
    flamegpu::ModelDescription model("V1.2");


    { // (optional local scope block for cleaner grouping)
        // Define a message of type MessageSpatial2D named location
        flamegpu::MessageSpatial2D::Description message = model.newMessage < flamegpu::MessageSpatial2D > ("information");
        // Configure the message list
        message.setMin(0, 0);
        message.setMax(ENV_WIDTH, ENV_WIDTH);
        message.setRadius(0.5f);
        // Add extra variables to the message
        // X Y (Z) are implicit for spatial messages
        message.newVariable < flamegpu::id_t > ("id");
        message.newVariable <int > ("neighbors");
        message.newVariable <int > ("type");
    }
    { // (optional local scope block for cleaner grouping)
        // Define a message of type MessageSpatial2D named location
        flamegpu::MessageSpatial2D::Description message2 = model.newMessage < flamegpu::MessageSpatial2D > ("location");
        // Configure the message list
        message2.setMin(0, 0);
        message2.setMax(ENV_WIDTH, ENV_WIDTH);
        message2.setRadius(0.5f);
        // Add extra variables to the message
        // X Y (Z) are implicit for spatial messages
        message2.newVariable < flamegpu::id_t > ("id");

        message2.newVariable <int > ("type");
    }

    { // (optional local scope block for cleaner grouping)
        // Define a message of type MessageSpatial2D named location
        flamegpu::MessageSpatial2D::Description message3 = model.newMessage < flamegpu::MessageSpatial2D > ("location2");
        // Configure the message list
        message3.setMin(0, 0);
        message3.setMax(ENV_WIDTH, ENV_WIDTH);
        message3.setRadius(0.5f);
        // Add extra variables to the message
        // X Y (Z) are implicit for spatial messages
        message3.newVariable < flamegpu::id_t > ("id");

        message3.newVariable <int > ("type");
    }
    // Define an agent named self-informed
    flamegpu::AgentDescription self = model.newAgent("self-informed");
    // Assign the agent some variables (ID is implicit to agents, so we don't define it ourselves)
    self.newVariable < float > ("x");
    self.newVariable < float > ("y");
    self.newVariable < float > ("vx");
    self.newVariable < float > ("vy");
    self.newVariable < int > ("neighbors");
    self.newVariable < int > ("type");

    // Define an agent named social
    flamegpu::AgentDescription social = model.newAgent("social");
    // Assign the agent some variables (ID is implicit to agents, so we don't define it ourselves)
    social.newVariable < float > ("x");
    social.newVariable < float > ("y");
    social.newVariable < float > ("vx");
    social.newVariable < float > ("vy");
    social.newVariable < int > ("neighbors");
    social.newVariable < int > ("type");

    // Define agent functions

    flamegpu::AgentFunctionDescription fn_message_social_out = social.newFunction("message_social_out", comm_social);
    fn_message_social_out.setMessageOutput("location");
    fn_message_social_out.setMessageInput("information");


    flamegpu::AgentFunctionDescription fn_message = self.newFunction("message", comm_informed);
    fn_message.setMessageInput("location");
    flamegpu::AgentFunctionDescription fn_message_out = self.newFunction("message_out", output_message);
    fn_message_out.setMessageOutput("information");




    // Define environment properties
    flamegpu::EnvironmentDescription env = model.Environment();
    env.newProperty<unsigned int>("AGENT_COUNT", AGENT_COUNT);
    env.newProperty<float>("ENV_WIDTH", ENV_WIDTH);

    // Define simulation layers


    // Add agent functions to layers
    //fn_message.dependsOn(fn_message_out2);
    fn_message.dependsOn(fn_message_social_out);
    fn_message_social_out.dependsOn(fn_message_out);
    model.addExecutionRoot(fn_message_out);
    //model.addExecutionRoot(fn_message_out2);
    model.generateLayers();




    model.addInitFunction(create_self);






    // Create the simulation
    flamegpu::CUDASimulation cuda_model(model, argc, argv);


    // Only compile this block if being built with visualisation support
#ifdef FLAMEGPU_VISUALISATION
    // Create visualisation
  flamegpu::visualiser::ModelVis m_vis = cuda_model.getVisualisation();


    flamegpu::visualiser::PanelVis ui = m_vis.newUIPanel("Settings");


  // Set the initial camera location and speed
  const float INIT_CAM = ENV_WIDTH / 2.0f;
  m_vis.setInitialCameraTarget(INIT_CAM, INIT_CAM, 0);
  m_vis.setInitialCameraLocation(INIT_CAM, INIT_CAM, ENV_WIDTH);
  m_vis.setCameraSpeed(0.01f);
  m_vis.setSimulationSpeed(24);
  // Add self-informed agents to the visualisation
  flamegpu::visualiser::AgentVis informed_agt = m_vis.addAgent("self-informed");

  // Location variables have names "x" and "y" so will be used by default
  informed_agt.setModel(flamegpu::visualiser::Stock::Models::ICOSPHERE);
  //head is 3 micrometer
  informed_agt.setModelScale(0.03f);

    // Add self-informed agents to the visualisation
  flamegpu::visualiser::AgentVis social_agt = m_vis.addAgent("social");
    social_agt.setColor(flamegpu::visualiser::Stock::Colors::RED);
  // Location variables have names "x" and "y" so will be used by default
 social_agt.setModel(flamegpu::visualiser::Stock::Models::ICOSPHERE);
  //head is 3 micrometer
  social_agt.setModelScale(0.03f);


     // Mark the environment bounds
    flamegpu::visualiser::LineVis pen = m_vis.newPolylineSketch(1, 1, 1, 0.2f);
    pen.addVertex(0, 0, 0);
    pen.addVertex(0, ENV_WIDTH, 0);
    pen.addVertex(ENV_WIDTH, ENV_WIDTH, 0);
    pen.addVertex(ENV_WIDTH, 0, 0);
    pen.addVertex(0, 0, 0);
  // Open the visualiser window
  m_vis.activate();
#endif

    // Run the simulation
    cuda_model.simulate();

#ifdef FLAMEGPU_VISUALISATION
    // Keep the visualisation window active after the simulation has completed
  m_vis.join();
#endif
}