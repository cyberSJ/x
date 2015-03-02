#ifndef dsBST_H
#define dsBST_H
#include <functional>
#include <cstddef> 
#include <iostream>
#include <algorithm>

/*
	From our last code, dsList.h, we built the data structure
	in the order of: declaring member variable, building constructor,
	iterating mechanism, and then controlling the data structure
	using the iterating mechanism.

	Let's apply this practice to dsBST.h.
*/

template<typename Object, typename Comparator>
class dsBST{

		/*
			[step2] Then it's time to have constructor.
			It should have an zero-parameter constructor (DONE: why?: because we are not putting any data to the data structure when we are creating a data structure. Just create the structure.)
			
		*/
	public:
		dsBST(){
			init();
		}

		/*
			[step5.1] Desctructor
		*/
		~dsBST(){
			makeEmpty(root);
		}

		/*
			[step3] It's time to make iterating mechanism, but in dsBST,
			there is no iterating mechanism, you travel while in the 
			BinaryNodes. Therefore, the next step is the static mechanism,
			i.e. contains(), findMin(), findMax()... These doesn't
			change the content of the data strucutre. These are accesors.
		*/

		/*
			contains().
			Should accept the obejct to find. Check if root is null,
			then proceed onto childs using recursive call.
			Should return bool
			But this public function use private version of contains().
			TODO: why should we do a wrap-up like this?
		*/
		bool contains(const Object &x) const{
			return contains(x, root);

		}

		void insert(const Object &x){
			insert(x, root);
		}

		void remove(const Object &x){
			remove(x, root);
		}

		/*
			printTree
			Accept ostream.
			Check if the tree is empty, and then print the 
			tree by calling on the root first.
			Inorder traversal. Returns nothing.
		*/
		void printTree(std::ostream & out = std::cout){
			if (isEmpty()){
				out << "Tree is empty." << std::endl;
			}
			else{
				printTree(root, out);
			}
		}

		int height(){
			return height(root);
		}
		



	private:
		/*
			[step1] Root has to be there. So I guess a pointer to a root?: YES.
			And we should have a pointer to the left and right child.
			But alternative is define the childs in the BinaryNode class.
			
			So what we learn is this: in defining a data structure,
			first thing to code is the building block of the data structure.
			In our case, this is BinaryNode. In dsList's case, it was
			a Node class that contains the data and prev/next pointers.
			And have the member variable be a pointer to that building
			block.
		*/

		/*
			BinaryNode struct.
			It should have a data, left/right child BinardyNode pointers.
			It also may contain frequency of the data?
			It should have a custom constructor.
		*/
		struct BinaryNode{
			Object element;	// TODO: why is it not a pointer?: because when we insert a new BinaryNode to the tree, we need to actually have a hard copy of the Object insdie the BinaryNode. But I think it is possible to have pointer to an Object instead of Object.
			BinaryNode *left;
			BinaryNode *right;

			BinaryNode(const Object &et, BinaryNode *lt, BinaryNode *rt) : element(et), left(lt), right(rt){		// and why the argument is a reference instead of copy-by-value?:

			}


		};

		/*
			Member variable
		*/
		BinaryNode *root;
		Comparator isLessThan;		// TODO: pointer to a Comparator...can't imagine for now... Let's do it:

		/*
			init.
			The root should be null.
			The left/right childs should be null.
			The root doesn't even have 
			any data yet because it's just an initialization.
		*/
		void init(){
			//root = new BinaryNode;
			root = NULL;	// If I don't set it to NULL, root will be not null, but some random garbage value.
		}

		/*
			[step3.1] private contains()
			Should accept the object to search for, and the tree to search
			in. Check if the root node is not null, then proceed to check
			for the left and right child using recursive call.
			Should return bool.
		*/
		//bool contains(const Comparable &x, dsBST *t){	// TODO: let's try using pointer to dsBST instead of root of some tree: It doesn't work since we are doing recursive style.
		bool contains(const Object &x, BinaryNode *r) const{
			if (r == NULL)	return false;
			//else if(x < r->element)	return contains(x, r->left);
			//else if(x > r->element) return contains(x, r->right);

			else if(isLessThan(x, r->element)) return contains(x, r->left);
			else if(isLessThan(r->element, x)) return contains(x, r->right);

			else return true;
		}

		/*
			[step3.2] findMin().
			Accept the BinaryNode to inspect.
			Just return the Object located at the far most
			left side of the tree. Don't forget to check for the null-ness
			of the root.
			Don't return the element, return the BinaryNode that contains
			the smallest element. This is also because of the recursive
			nature.
		*/
		BinaryNode * findMin(BinaryNode *t) const{
			if (t == NULL) return NULL;
			if (t->left == NULL) return t;
			return findMin(t->left);

			// TODO: Let's try the code below instead.
			//if (t->element == NULL) return NULL;
			//if (t->left != NULL) return findMin(t->left);	// TODO: Let's put else if instead of if.
			//else return &t->left;
		}

		/*
			[step3.3] findMax().
			Non-recursive function.
			Accepts the root node of the tree
			Still return the pointer to the BinaryNode
			that contains the maximum element
		*/
		BinaryNode * findMax(BinaryNode *t) const{
			if (t == NULL) return false;

			while (t->right != NULL){
				t = t->right;
			}
			return t;
		}

		/*
			[step4] Then we create dynamic functions.
		*/
		
		/*
			[step4.1] insert().
			Accept an object to insert, travel down the tree by using the
			comparator, and create a new pointer to a BinaryNode that
			contains that data. Also accept a BinaryNode pointer since
			this function is recursive.
			returns void
		*/
		void insert(const Object &x, BinaryNode *&t){
			if (t != NULL){
				if (isLessThan(x, t->element)) insert(x, t->left);
				else if (isLessThan(t->element, x)) insert(x, t->right);
			}
			else{
				t = new BinaryNode(x, NULL, NULL);
			}
		} 

		/*
			[step4.2] remove
			Accept the Object to remove and the Subtree to search for.
			If we find the Object match, then remove the BinaryNode,
			then find the left-most BinaryNode in the right subtree,
			then replace the delete node with that new BinaryNode.
			Return void
		*/
		void remove(const Object &x, BinaryNode *&t){ // DONE: why *&?: because we need to work with the real (not a copied version) pointer passed to the argument of the function. *& is not necessary in remove(), but it is required for insert() because we need to modify the content of the pointer (different than modifying the content of the obejct that's pointed by the pointer.
			if (t == NULL) return;
			if (isLessThan(x, t->element)) remove(x, t->left);
			else if (isLessThan(t->element, x)) remove(x, t->right);
			else if (t->left != NULL && t->right != NULL){
				// There are two children, and we need to replace this node.
				t->element = findMin(t->right)->element;
				remove(t->element, t->right);
			}
			else {
				// This code takes care of AFTER the above elseif block 
				// has been executed.
				BinaryNode *oldNode = t;
				t = (t->left != NULL) ? t->left: t->right;
				delete oldNode;
			}
		}

		/*
			[step5] Need to make desctructor
		*/
		
		/*
			[step5.2] makeEmpty().
			
		*/
		void makeEmpty(BinaryNode *&t){
			if (t != NULL){
				makeEmpty(t->left);
				makeEmpty(t->right);
				delete t;
			}
			// If we're here then t is already equal to NULL
			// DONE: so we don't need "t = NULL" ??: Yeah it works.
			t = NULL;
		}

		bool isEmpty(){
			return root == NULL;
		}

		void printTree(const BinaryNode *t, std::ostream &out) const{
			if (t != NULL){
				printTree(t->left, out);
				out << t->element << std::endl;
				printTree(t->right, out);
			}
		}

		int height(const BinaryNode *t){
			if (t == NULL){
				return -1;
			}
			else{
				return std::max(height(t->left), height(t->right)) + 1;
			}
		}	
		

		
		
		
		



};



#endif
