package searchclient;

import java.util.Comparator;
import java.util.LinkedList;
import java.util.Vector;
import java.lang.Math;
import java.util.ArrayList;

import searchclient.NotImplementedException;

public abstract class Heuristic implements Comparator<Node> {
	private int distances[][][];
	// private Vector<Int[]> goals;
	private ArrayList<int[]> goals;


	public Heuristic(Node initialState) {
		/****** Preprocessing ******/
		// Calculating the Manhatten distance for cells to goals
		// 0.) Set list of goal states coords and values
		this.goals = new ArrayList<int[]>();
		// Vector goalChars  = new Vector();
		for (int row = 0; row < initialState.MAX_ROW; row++) {
			for (int col = 0; col < initialState.MAX_COL; col++) {
				if ((int)initialState.goals[row][col] > 0) {
					int[] nP = {row, col, (int)initialState.goals[row][col] - 32};
					this.goals.add(nP);
				}
			}
		}		
		// For each goal
		this.distances  = new int [initialState.MAX_ROW][initialState.MAX_COL][this.goals.size()]; // not sure I can dynamically set this -- may have to put in the node thing
		for (int row = 0; row < initialState.MAX_ROW; row++) {
			for (int col = 0; col < initialState.MAX_COL; col++) {
				// if the node is a wall, continue
				if (initialState.walls[row][col]) {continue;}
				// Loop through each goal and calculate the distance to the cell
				for (int goal = 0; goal < this.goals.size(); goal++) {
					this.distances[row][col][goal] = Math.abs(row - this.goals.get(goal)[0]) + Math.abs(col - this.goals.get(goal)[1]);
				}
			}
		}
	}

	public int h(Node n) {
		// Heuristic function here
		int h_val = 0;
		for (int row = 0; row < n.MAX_ROW; row++) {
			for (int col = 0; col < n.MAX_COL; col++) {
				if (!n.boxAt(row, col)) {continue;} // if there is not a box continue
				// If there is, determine which character it is
				// Then match it with a goal state
				for (int goal = 0; goal < this.goals.size(); goal++) {
					if ((int)n.boxes[row][col] == this.goals.get(goal)[2]) {
						h_val += this.distances[row][col][goal];	
					}
				}
			}
		}
		return h_val;
	}

	public abstract int f(Node n);

	@Override
	public int compare(Node n1, Node n2) {
		return this.f(n1) - this.f(n2);
	}

	public static class AStar extends Heuristic {
		public AStar(Node initialState) {
			super(initialState);
		}

		@Override
		public int f(Node n) {
			return n.g() + this.h(n);
		}

		@Override
		public String toString() {
			return "A* evaluation";
		}
	}

	public static class WeightedAStar extends Heuristic {
		private int W;

		public WeightedAStar(Node initialState, int W) {
			super(initialState);
			this.W = W;
		}

		@Override
		public int f(Node n) {
			return n.g() + this.W * this.h(n);
		}

		@Override
		public String toString() {
			return String.format("WA*(%d) evaluation", this.W);
		}
	}

	public static class Greedy extends Heuristic {
		public Greedy(Node initialState) {
			super(initialState);
		}

		@Override
		public int f(Node n) {
			return this.h(n);
		}

		@Override
		public String toString() {
			return "Greedy evaluation";
		}
	}
}
