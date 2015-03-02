#ifndef dsBinomialQ_H
#define dsBinomialQ_H

#include <vector>
#include <iostream>
#include <algorithm>

template<typename Comparable>
class dsBinomialQ{

	public:
		dsBinomialQ(){
		
		}

		dsBinomialQ(const Comparable &x):currentSize(1){
			BinomialNode *bp = new BinomialNode(x);
			forest.push_back(bp);
		}
		
		bool isEmpty() const{
			return currentSize == 0;
		}


		/*
			This method has problems:
			1. forest[0] might be empty in BinomialQ
			2. forest[i]->element < min
				operator< might be unsuitable for comparing Comparalbe vs. NULL
			3. Comparable objects are copied everytime a new minimum is found. Not efficient.
			4. Sub-routine can be formed called findMinIndex(). Refer to pg.250
		*/
		//const Comparable &findMin() const{
		//	Comparable min = NULL;
		//	if (!isEmpty()){
		//		min = forest[0]->element;
		//		for (int i = 0; i < forest.size(); i++)
		//			if (forest[i] != NULL && forest[i]->element < min)
		//				min = forest[i]->element;
		//	}

		//	/*
		//		Return value must be either:
		//		1. null if the currentSize == 0
		//		2. minimum value in the BionmialQ.
		//	*/
		//	return min;
		//}
		
		const Comparable & findMin() const{
			int minIndex = findMinIndex();
			return forest[minIndex]->element;
		}

		/*
			insert.
			Insert is just one case of merge. One node merge.
		*/
		void insert(const Comparable &x){
			dsBinomialQ tempQ(x);
			merge(tempQ);
		}

		/*
			merge.
			Too complicated. Doing this in Senegal.
		*/
		void merge(dsBinomialQ & rhs){
			if (this == &rhs){
				return;
			}

			currentSize += rhs.currentSize;		// Combine sizes of the two Qs.

			// Expand if necessary.
			if (currentSize > capacity()){
				int oldNumTrees = forest.size();
				int newNumTrees = std::max(forest.size(), rhs.forest.size()) + 1;	// Largest forest + 1
				forest.resize(newNumTrees);
				for (int i = oldNumTrees; i < newNumTrees; i++)
					forest[i] = NULL;
			}

			BinomialNode *carry = NULL;
			for (int i = 0, j = 1; j <= currentSize; i++, j *= 2){	// Technique for using a for loop with power incrementation.
				BinomialNode *t1 = forest[i];	// I'm guaranteed to be able to access forest[i].
				BinomialNode *t2 = i < rhs.forest.size() ? rhs.forest[i] : NULL;	// But this time I need to check if I can access rhs.forest[i]

				// whichCase is a binary number.
				int whichCase = t1 == NULL ? 0 : 1;
				whichCase += t2 == NULL ? 0 : 2;
				whichCase += carry == NULL ? 0 : 4;
				
				switch(whichCase){
					case 0: /* No trees */
					//	break;
					case 1:	/* Only this */
						break;
					case 2: /* Only rhs */
						forest[i] = t2;	// there's only H2's tree to merge
						rhs.forest[i] = NULL;	// move (not copy)
						break;
					case 4:	/* Only carry */
						forest[i] = carry;	// carry is a tree combined from previous iteration i.
						carry = NULL;		// Since we put carry in this iteration, we don't have any carry in the next iteration.
						break;
					case 3: /* this and rhs */
						carry = combineTrees(t1, t2);	// combine trees, but don't store the combined tree in i'th slot. But store it as a carry so that I can store the carry in the next forest slot.
						forest[i] = rhs.forest[i] = NULL;	// Make sure to empty the trees that were used for combineTrees().
						break;
					case 5: /* this and carry */
						carry = combineTrees(t1, carry);
						forest[i] = NULL;
						break;
					case 6: /* rhs and carry */
						carry = combineTrees(t2, carry);
						rhs.forest[i] = NULL;
						break;
					case 7: /* all three */
						forest[i] = carry;	// use the carry first.
						carry = combineTrees(t1, t2);
						rhs.forest[i] = NULL;
						break;
				}
			}

			for (int k = 0; k < rhs.forest.size(); k++){
				rhs.forest[k] = NULL;
			}
			rhs.currentSize = 0;
		}

		void deleteMin(Comparable &minItem){
			if (isEmpty()){
				throw UnderflowExeption();
			}
			int minIndex = findMinIndex();
			minItem = forest[minIndex]->element;

			// Save the next heir.
			BinomialNode *oldRoot = forest[minIndex];
			BinomialNode *deletedTree = oldRoot->leftChild;
			delete oldRoot;
	
			// Construct H" and decompose the deleted tree.
			dsBinomialQ deletedQueue;
			deletedQueue.forest.resize(minIndex + 1);	// +1 because minIndex is an index but we want size. (size = index + 1)
			deletedQueue.currentSize = (1 << minIndex) - 1; 	// You know the currentSize because the size of one tree located at any index can be known. -1 because we are not including the root of the tree pointed by the minIndex to the deletedQueue.
			for (int j = minIndex - 1; j >= 0; j--){
				deletedQueue.forest[j] = deletedTree;
				deletedTree = deletedTree->nextSibling;
				deletedQueue.forest[j]->nextSibling = NULL;
			}

			// Construct H' which is a Binomial queue w/o the tree pointed by the minIndex
			forest[minIndex] = NULL;
			currentSize -= deletedQueue.currentSize + 1;		// TODO: I wonder about precedence let's try currentSize = currentSize - (deletedQueue.currentSize + 1); and w/o the parenthesis.
			
			merge(deletedQueue);
		}


	private:
		struct BinomialNode{
			Comparable element;
			BinomialNode *leftChild;
			BinomialNode *nextSibling;

			BinomialNode(const Comparable &el = Comparable(), BinomialNode *lc = NULL, BinomialNode *ns = NULL)
				: element(el), leftChild(lc), nextSibling(ns){
			}
		};

		std::vector<BinomialNode *> forest;
		int currentSize;

		int findMinIndex() const{
			int i;
			int minIndex;

			// find the first non-null index
			for (i = 0; forest[i] == NULL; i++)
				;	
			
			// find the minimum index using two index keepers.
			for (minIndex = i; i < forest.size(); i++){
				if (forest[i] != NULL && forest[i]->element < forest[minIndex]->element)
					minIndex = i;
			}
			
			return minIndex;
		}
		
		int capacity() const{
			int capacity = 0;
			for (int i = 0; i < forest.size(); i++)
				capacity += 1 << i;
			return capacity;
		}

		BinomialNode * combineTrees(BinomialNode *t1, BinomialNode *t2){
			if (t2->element < t1->element){
				return combineTrees(t2, t1);
			}
			t2->nextSibling = t1->leftChild;
			t1->leftChild = t2;
			return t1;
		}

		int UnderflowExeption(){
			std::cout << "Error: underflow exeption!" << std::endl;
			return 1;
		}
		

};

#endif
