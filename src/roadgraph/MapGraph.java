/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	private Map<GeographicPoint,MapNode> vertices;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		this.vertices = new HashMap<GeographicPoint,MapNode>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return vertices.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		return vertices.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		int ne = 0;
		for (MapNode mn : vertices.values()) {
			ne += mn.getNumNeighbors();
		}
		return ne;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		if (vertices.containsKey(location)) {
			return false;
		} else {
			vertices.put(location, new MapNode(location));
			return true;
		}
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		if (from == null || to == null || 
				!vertices.containsKey(from) ||
				!vertices.containsKey(to) ||
				roadName == null || roadType == null ||
				length < 0) {
			throw new IllegalArgumentException();
		}
		
		vertices.get(from).addEdge(vertices.get(to), roadName, roadType, length);
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		if (start == null || goal == null || 
				!vertices.containsKey(start) ||
				!vertices.containsKey(goal)) {
			return null;
		}

		Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		boolean found = bfsSearch(start, goal, parentMap, nodeSearched);

		if (found) {
			return constructPath(start, goal, parentMap);
		} else {
			return null;
		}
	}
	
	/** Search loop for breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param parentMap The mapping between nodes and the node that led to them - to be filled in
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return True/False that a path was found. ParentMap is filled up.
	 */
	private boolean bfsSearch(GeographicPoint start, GeographicPoint goal,
			Map<GeographicPoint, GeographicPoint> parentMap,
			Consumer<GeographicPoint> nodeSearched) {
		List<GeographicPoint> queue = new LinkedList<GeographicPoint>();
		Set<GeographicPoint> visited = new HashSet<GeographicPoint>();
		
		queue.add(start);
		visited.add(start);
		
		while (!queue.isEmpty()) {
			// remove first in queue
			GeographicPoint cur = queue.remove(0);
			// if current is goal, exit
			if (cur.equals(goal)) return true;
			
			// add current's neighbors to queue and visited and parent map
			for (MapEdge e : vertices.get(cur).getNeighbors()) {
				GeographicPoint p = e.getEnd().getLocation();
				if (!visited.contains(p)) {
					queue.add(p);
					visited.add(p);
					parentMap.put(p, cur);
					nodeSearched.accept(p);
				}
			}
		}
		
		// goal not found
		return false;
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		if (start == null || goal == null || 
				!vertices.containsKey(start) ||
				!vertices.containsKey(goal)) {
			return null;
		}
		
		// initialize all distances to infinity
		for (MapNode mn : vertices.values()) {
			mn.setDistanceFromStart(Double.POSITIVE_INFINITY);
		}

		Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		boolean found = DijkstraSearch(start, goal, parentMap, nodeSearched);

		if (found) {
			return constructPath(start, goal, parentMap);
		} else {
			return null;
		}
	}

	private boolean DijkstraSearch(GeographicPoint start, GeographicPoint goal,
			Map<GeographicPoint, GeographicPoint> parentMap, Consumer<GeographicPoint> nodeSearched) {
		PriorityQueue<MapNode> queue = new PriorityQueue<MapNode>(new DijkstraComparator());
		Set<GeographicPoint> visited = new HashSet<GeographicPoint>();
		
		vertices.get(start).setDistanceFromStart(0);
		queue.add(vertices.get(start));
		visited.add(start);
		
		while (!queue.isEmpty()) {
			// get next node, closest in terms of distance from start
			// add to visited and nodeSearched
			MapNode cur = queue.remove();
			visited.add(cur.getLocation());
			nodeSearched.accept(cur.getLocation());
			
			// exit if this is the goal
			if (cur.getLocation().equals(goal)) return true;
			
			// loop over all neighbors
			for (MapEdge e : cur.getNeighbors()) {
				// if unvisited and this is a shorter path to the node, then add to queue
				if (!visited.contains(e.getEnd().getLocation()) &&
						cur.getDistanceFromStart() + e.getLength() < e.getEnd().getDistanceFromStart()) {
					// set new distance
					e.getEnd().setDistanceFromStart(cur.getDistanceFromStart() + e.getLength());
					// set parent
					parentMap.put(e.getEnd().getLocation(), cur.getLocation());
					// add to queue
					queue.add(e.getEnd());
				}
			}
		}
		
		// goal not found
		return false;
	}

	private class DijkstraComparator implements Comparator<MapNode> {
		@Override
		public int compare(MapNode o1, MapNode o2) {
			if (o1.getDistanceFromStart() < o2.getDistanceFromStart()) {
				return -1;
			} else if (o1.getDistanceFromStart() > o2.getDistanceFromStart()) {
				return 1;
			} else {
				return 0;
			}
		}
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		if (start == null || goal == null || 
				!vertices.containsKey(start) ||
				!vertices.containsKey(goal)) {
			return null;
		}
		
		// initialize all distances to infinity
		for (MapNode mn : vertices.values()) {
			mn.setDistanceFromStart(Double.POSITIVE_INFINITY);
			mn.setEstDistFromGoal(Double.POSITIVE_INFINITY);
		}

		Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		boolean found = AStarSearch(start, goal, parentMap, nodeSearched);

		if (found) {
			return constructPath(start, goal, parentMap);
		} else {
			return null;
		}
	}

	private boolean AStarSearch(GeographicPoint start, GeographicPoint goal,
			Map<GeographicPoint, GeographicPoint> parentMap, Consumer<GeographicPoint> nodeSearched) {
		PriorityQueue<MapNode> queue = new PriorityQueue<MapNode>(new AStarComparator());
		Set<GeographicPoint> visited = new HashSet<GeographicPoint>();
		
		vertices.get(start).setDistanceFromStart(0);
		vertices.get(start).setEstDistFromGoal(start.distance(goal));
		queue.add(vertices.get(start));
		visited.add(start);
		
		while (!queue.isEmpty()) {
			// get next node, closest in terms of distance from start
			// add to visited and nodeSearched
			MapNode cur = queue.remove();
			visited.add(cur.getLocation());
			nodeSearched.accept(cur.getLocation());
			
			// exit if this is the goal
			if (cur.getLocation().equals(goal)) return true;
			
			// loop over all neighbors
			for (MapEdge e : cur.getNeighbors()) {
				// calculate estimated distance from neighbor to goal
				e.getEnd().setEstDistFromGoal(e.getEnd().getLocation().distance(goal));
				// if unvisited and this is a shorter path, then add to queue
				if (!visited.contains(e.getEnd().getLocation()) &&
						cur.getDistanceFromStart() + e.getLength() + e.getEnd().getEstDistFromGoal() 
						< e.getEnd().getTotalDistance()) {
					// set new distance from start
					e.getEnd().setDistanceFromStart(cur.getDistanceFromStart() + e.getLength());
					// set parent
					parentMap.put(e.getEnd().getLocation(), cur.getLocation());
					// add to queue
					queue.add(e.getEnd());
				}
			}
		}
		
		// goal not found
		return false;
	}

	private class AStarComparator implements Comparator<MapNode> {
		@Override
		public int compare(MapNode o1, MapNode o2) {
			if (o1.getTotalDistance() < o2.getTotalDistance()) {
				return -1;
			} else if (o1.getTotalDistance() > o2.getTotalDistance()) {
				return 1;
			} else {
				return 0;
			}
		}
	}

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param parentMap The mapping between nodes and the node that led to them - now filled in
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	private List<GeographicPoint> constructPath(GeographicPoint start, GeographicPoint goal,
			Map<GeographicPoint, GeographicPoint> parentMap) {
		List<GeographicPoint> path = new LinkedList<GeographicPoint>();
		path.add(goal);
		while (!path.get(0).equals(start)) {
			path.add(0, parentMap.get(path.get(0)));
		}
		return path;
	}

	public static void main(String[] args)
	{
		System.out.println("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.println("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		/* Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
