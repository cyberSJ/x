#ifndef dsLeftistHeap_H
#define dsLeftistHeap_H

#include <iostream>

template<typename Comparable>
class dsLeftistHeap{

	public:
		/*
			[step2] Constructor
		*/
		dsLeftistHeap(){
			root = NULL;	// If don't set root to NULL, root will be non-null and mess up the algorithm later on.
		}

		/*
			[step3] Accessor
		*/
		bool isEmpty() const{
			return root == NULL;
		}

		const Comparable &findMin() const{
			return root->element;
		}

		/*
			[step5] Mutator
		*/
		/*
			insert.
			Duplicates allowed.
		*/
		void insert(const Comparable &x){
			root = merge(new LeftistNode(x), root);
		}

		/*
			deleteMin.
			Just deletes it.
		*/
		void deleteMin(){
			if (isEmpty())	throw UnderflowException();

			LeftistNode *oldRoot = root;			// Save the old root.
			root = merge(oldRoot->left, oldRoot->right); 	// Define new root.
			delete root;					// Delete the old root that we SAVED.
		}

		void deleteMin(Comparable &x){
			x = findMin();
			deleteMin();
		}

		void merge(dsLeftistHeap &rhs){
			// Self-checking: h.merge(h) must not be possible.
			if (this == &rhs)	return;

			root = merge(root, rhs.root);
			rhs.root = NULL;	// rhs is not a root anymore, but just another LeftistNode.
		}

	


	private:

		/*
			[step1] Data Structure Building Block
			LeftistNode struct.
		*/
		struct LeftistNode{
			Comparable element;
			LeftistNode *left;
			LeftistNode *right;
			int npl;		// Null Path Length

			LeftistNode(const Comparable &theElement, LeftistNode *lt = NULL, LeftistNode *rt = NULL, int np = 0)
				: element(theElement), left(lt), right(rt), npl(np){

			}
		};

		LeftistNode *root;

		/*
			[step4.1] merge.
			Internal method to merge two roots.
			Deals with deviant cases and calls recursive merge1.
		*/
		LeftistNode *merge(LeftistNode *h1, LeftistNode *h2){
			// Check for emptiness first.
			if (h1 == NULL)	return h2;
			if (h2 == NULL) return h1;			

			// Decide which root has smaller value.
			if (h1->element < h2->element)
				return merge1(h1, h2);
			else
				return merge1(h2, h1);
		}

		/*
			[step4.2] merge1.
			The real workforce in merging two roots.
			Assumes trees are not empty, and h1's root contains the smallest item.
			Refer to pg.235
			Return LeftistNode pointer, which is the root of the new
			merged tree.
		*/
		LeftistNode *merge1(LeftistNode *h1, LeftistNode *h2){
			if (h1->left == NULL)	// This means there's no child
				h1->left = h2;		// Don't need to update the npl because npl stays the same in this situation.
			else{
				h1->right = merge(h1->right, h2);
				if (h1->left->npl < h1->right->npl){
					swapChildren(h1);
				}
				h1->npl = h1->right->npl + 1;	// After swapChildren, the right child's npl will be always smaller than that of the left child.
			}
			return h1;
		}
		
		void swapChildren(LeftistNode *h){
			LeftistNode *temp = h->left;
			h->left = h->right;
			h->right = temp;
		}

		int UnderflowException(){

			std::cout << "Error: the Leftistheap is empty!" << std::endl;
			return 1;
		}


		

};


#endif
