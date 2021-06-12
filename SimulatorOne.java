import java.util.List;
import java.util.Map;
import java.util.LinkedList;
import java.util.HashMap;
import java.util.NoSuchElementException;
import java.util.PriorityQueue;
import java.util.Scanner;
import java.util.StringTokenizer;
/**
 * This assignment concerns using directed graphs to develop an algorithm for a new business called UberHaul.
 *  As the name suggests, it is similar to Uber, but for moving materials, not people. 
 * Clients who need items taken by truck from one location to another can request an UberHaul driver.
 * Clients are people who ask for an UberHaul driver to take goods from one node to another node.
 * Edges in the graph represent routes the drivers take, modelled as a weighted directed graph.
 * Nodes represent locations in the city. Some are where UberHaul trucks are stationed, 
 * some are the source or destination of deliveries, 
 * and some are just intersections where drivers can change direction to minimise cost.
 * @author Madikizela Will
 * @version 2021/06/12
 */

    class GraphException extends RuntimeException {
        private static final long serialVersionUID = 1L;

        public GraphException( String name ) {
            super( name );
        }
    }
    
    class Edge {
        public Node destination;
        public double cost;  

        public Edge( Node d, double c ){
            destination = d;
            cost = c;
        }
    }

    class Path implements Comparable<Path> {
        public Node     destination;  
        public double     cost;

        public Path( Node d, double c ) {
            destination = d;
            cost = c;
        }

        public int compareTo( Path path ) {
            double otherCost = path.cost;

            if (cost < otherCost){
                return -1;
            }
            else if ( cost > otherCost){
                return 1;
            }
            else return 0;

        }
    }

    class Node {
        /**
         * name --> vertex name
         * adj --> adjecent vertices 
         * dist --> cost
         * prev --> Previous vertex on shortest path
         * scratch --> Extra variable used in algorithm
         */
        public String     name;
        public List<Edge> adj;
        private double     dist;
        public Node     prev; 
        public boolean    scratch;
        public boolean    allow;

        public Node( String otherName )
        {allow = false; name = otherName; adj = new LinkedList<Edge>( ); reset( ); }

        public void reset( )
        { setDist(SimulatorOne.INFINITY); prev = null; scratch = false; }

        public double getDist() {
            return dist;
        }

        public void setDist(double dist) {
            this.dist = dist;
        }
    }

    public class SimulatorOne {

        boolean state = false;
        public static final double INFINITY = Double.MAX_VALUE;
        private Map<String, Node> vertexMap = new HashMap<String, Node>();

        public void addEdge(String sourceName, String destinationName, double cost) {
            Node sourceVertex = getVertex(sourceName);
            Node destinationVertex = getVertex(destinationName);
            sourceVertex.adj.add(new Edge(destinationVertex, cost));
        }
        public void printPath(String destinationName) {
            Node destinationVertex = vertexMap.get(destinationName);
            if (destinationVertex == null)
                throw new NoSuchElementException("destination vertex not found");
            else if (destinationVertex.getDist() == INFINITY)
                System.out.println("cannot be helped");
            else {
                printPath(destinationVertex);
                System.out.println();
            }
        }
        private Node getVertex(String vertexName) {
            Node sourceVertex = vertexMap.get(vertexName);
            if (sourceVertex == null) {
                sourceVertex = new Node(vertexName);
                vertexMap.put(vertexName, sourceVertex);
            }
            return sourceVertex;
        }
        private void printPath(Node destination) {
            if (destination.prev != null) {
                printPath(destination.prev);
                System.out.print(" ");
            }
            System.out.print(destination.name);
        }
        private void clearAll() {
            for (Node sourceVertex : vertexMap.values())
                sourceVertex.reset();
        }


        public void dijkstra(String source) throws NullPointerException {
            PriorityQueue<Path> Queue = new PriorityQueue<Path>();

            Node start = vertexMap.get(source);
            if (start == null) {
                System.out.println("cannot be helped");
                state = true;
            }
            if (this.state == false){
            clearAll();
            final boolean add = Queue.add(new Path(start, 0));
            assert start != null;
            start.setDist(0);

            int nodesSeen = 0;
            while (!Queue.isEmpty() && nodesSeen < vertexMap.size()) {
                Path vrec = Queue.remove();
                Node sourceVertex = vrec.destination;
                if (sourceVertex.scratch != false) 
                    continue;

                sourceVertex.scratch = true;
                nodesSeen++;
                for (Edge edge : sourceVertex.adj) {
                    Node destinationVertex = edge.destination;
                    double weight = edge.cost;

                    if (weight < 0)
                        throw new GraphException("Graph has negative edges");
                    if (destinationVertex.getDist() > sourceVertex.getDist() + weight) {
                        destinationVertex.setDist(sourceVertex.getDist() + weight);
                        destinationVertex.prev = sourceVertex;
                        Queue.add(new Path(destinationVertex, destinationVertex.getDist()));
                        destinationVertex.allow = false;
                    } else if (destinationVertex.getDist() == sourceVertex.getDist() + weight) {
                        destinationVertex.allow = true;

                    }
                }
                }
            }
        }

        public static void main(String[] args) {
            SimulatorOne truckToDropOff = new SimulatorOne();
            SimulatorOne truckToPickUp = new SimulatorOne();
            SimulatorOne truckToHome = new SimulatorOne();
            Scanner input = new Scanner(System.in);
            String lineOne = input.nextLine(); 
            int numNodes = Integer.parseInt(lineOne); 
            /**
             * read all edges and store them 
             * iterate through all nodes and edges 
             * add nodes to graph 
             * add edge to graph 
             * get cost of each destination 
             */
            String line;
            for (int i = 0; i < numNodes; i++) {
                line = input.nextLine();
                StringTokenizer token = new StringTokenizer(line);
                String source = token.nextToken();
                while (token.countTokens() != 0) {   
                    String destination = token.nextToken(); 
                    int cost = Integer.parseInt(token.nextToken()); 
                    truckToDropOff.addEdge(source, destination, cost); 
                    truckToPickUp.addEdge(source, destination, cost);
                    truckToHome.addEdge(source, destination, cost);
                }
            }
            int numberOfDrivers = Integer.parseInt(input.nextLine());
            String DriverHomes = input.nextLine();
            int numberOfRequests = Integer.parseInt(input.nextLine());
            String RequestRoutes = input.nextLine();
            StringTokenizer routes = new StringTokenizer(RequestRoutes);
            System.out.println();
            String[] sources = new String[numberOfRequests];
            String[] destinations = new String[numberOfRequests];
            /**
             * go through all requests
             * store all costs for all drivers 
             * get the cheapest cost 
             */

            for (int j = 0; j < numberOfRequests; j++) { 
                double pickup = 0;
                double deliver = 0;
                double homecost = 0;
                String source = routes.nextToken();
                sources[j] = source;
                String destination = routes.nextToken();
                destinations[j] = destination;

                System.out.println("client " + source + " " + destination);
                if (source == null) {
                    System.out.println("cannot be helped 5");
                    break;
                }
                StringTokenizer Homes = new StringTokenizer(DriverHomes);

                double lowest = 99999;
                String driverNumber = "";
                String destinationName = "";
                boolean state2 = false;

                for (int i = 0; i < numberOfDrivers; i++) {
                    destinationName = Homes.nextToken();
                    String Driverhome = destinationName;

                    truckToPickUp.dijkstra(Driverhome);
                    if (truckToPickUp.state){
                        continue;
                    }
                    Node pickUpNode = truckToPickUp.getVertex(source);
                    pickup = pickUpNode.getDist();
                    truckToHome.dijkstra(destination);
                    if (truckToHome.state){
                        continue;
                    }
                    Node endNode = truckToHome.getVertex(Driverhome);
                    homecost = endNode.getDist();
                    double driverCost = homecost + pickup;

                    if (driverCost < lowest) {
                        lowest = driverCost;
                        driverNumber = Driverhome;
                    } else if (driverCost == lowest) { 
                        state2 = true;
                        if (Integer.parseInt(Driverhome) < Integer.parseInt(driverNumber)) {
                            driverNumber = Driverhome;
                        }
                    }
                }
                //cheapest driver
                truckToPickUp.dijkstra(driverNumber); 
                if (truckToPickUp.state){
                    continue;
                }
                // cheapest route home
                truckToHome.dijkstra(destination); 
                if (truckToHome.state){
                    continue;
                }
                // find the cheapest delivery route
                truckToDropOff.dijkstra(source); 
                if (truckToDropOff.state){
                    continue;
                }
                if (!(truckToDropOff.state == true || truckToPickUp.state == true || truckToHome.state == true)) {
                    System.out.println("truck " + driverNumber);
                    Node sourceNode = truckToPickUp.getVertex(source);
                    if (sourceNode.allow == true) {
                        System.out.println("multiple solutions cost " + (int) sourceNode.getDist());
                    } else {
                        truckToPickUp.printPath(source);
                    }
                    Node DropNode = truckToDropOff.getVertex(destination);
                    int dropcost = (int) DropNode.getDist();
                    System.out.println("pickup " + source);
                    if (DropNode.allow == true) {
                        System.out.println("multiple solutions cost " + dropcost);
                    } else {
                        truckToDropOff.printPath(destination);
                    }
                    System.out.println("dropoff " + destination);
                    Node destinationNode = truckToHome.getVertex(destination);
                    if (destinationNode.allow == true) {
                        System.out.println("multiple solutions cost " + (int) destinationNode.getDist());
                    } else if (truckToHome.state == false) {
                        truckToHome.printPath(driverNumber);
                    }
                }
            }
        }
    }