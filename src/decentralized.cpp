
// Decentralized Constructor

//// Call run



// Decentralized::Run

//// Load YAML / Create Topology

//// Create Node Instances Per Node

//// Mesh Nodes

//// Loop through nodes

////// Create FG

////// Solve FG

////// Transmit other data to next - just save factors



// Node Constructor

// Create FG

////



// PropagationEnvironment -- to be handled fully later. for now can just be a link





// Focus of code:

// Create many node instances
//          Each Node Represents A Comms Node
//          They will identify their peers / mesh strengths, etc -- eventually noise added there
//          They will create an LQR based factor graph based on their local problem
//          They will solve local problem leaving behind priors
//          Priors sent as message to next node which generates factor graph, solves, etc



// Architecture:

// Class DecentralizedSimulator
    // Create Topology (/ Load YAML With Locations)

// Class Node
    // Number / ID
    // Location

    // Create FG
    // Solve FG

// Class Link
    // Node 1
    // Node 2

    // Transmit










