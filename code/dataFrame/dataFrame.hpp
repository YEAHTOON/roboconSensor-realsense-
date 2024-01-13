#ifndef __DATAFRAME_H_
#define __DATAFRAME_H_

#include <vector>

typedef struct threeD_point
{
    float x;
    float y;
    float z;
} threeD_point;


/**
 * 队列节点
*/
template<typename T>
class queueNode
{
public:
    queueNode()
    {
        this->data = nullptr;
        this->nextNode = nullptr;
    }

    queueNode(T theData)
    {
        this->data = theData;
        this->nextNode = nullptr;
    }

    ~queueNode()
    {
        // delete this->data;
    }

    queueNode *nextNode;
    T data;
};




/**
 * 队列
*/
template<typename T>
class queue
{
public:
    queue()
    {
        this->front = nullptr;
        this->rear = nullptr;
    }

    queue(queueNode<T> *theFront, queueNode<T> *theRear)
    {
        this->front = theFront;
        this->front->nextNode = nullptr;
        this->rear = theRear;
        this->rear->nextNode = nullptr;
    }
    
    ~queue()
    {
        while(this->front != nullptr)
        {
            this->pop();
        }
    }

    void pop()
    {
        if(this->front != nullptr)
        {
            queueNode<T> *lastFront = this->front;
            this->front = this->front->nextNode;
            if(this->front == nullptr) this->rear = nullptr;
            delete lastFront;
            lastFront = nullptr;
            // std::cout << this->front << std::endl;
        }
    }

    //插入新元素
    void push(queueNode<T> *newNode)
    {
        if(this->rear != nullptr)
        {
            this->rear->nextNode = newNode;
            this->rear = newNode;
        } 
        else
        {
            this->rear = newNode;
            this->front = newNode;
        }
    }

    //队列是否为空
    bool ifEmpty(void)
    {
        if(this->rear == nullptr)
        {
            return true;
        }
        if(this->front == nullptr)
        {
            return true;
        }
        return false;
    }

    queueNode<T> *front;
    queueNode<T> *rear;
};




/**
 * 一个点集
*/
class potentialStraightLine
{
public:

    potentialStraightLine(int k, int y)
    {
        this->kCount = 1;
        this->k = k;
        this->highest = y;
        this->lowest = y;
    }

    void pushK(int k, int y)
    {
        if(y > this->highest) this->highest = y;
        if(y < this->lowest) this->lowest = y;
        this->k = (this->kCount * this->k + k) / (kCount + 1);
        this->kCount++;
    }

    float k;
    int kCount;
    int highest;
    int lowest;
};




/**
 * 链表节点
*/
template<typename theType>
class linkNode
{
public:

    linkNode(theType data)
    {
        this->data = data;
    }
    theType data;
    linkNode *next;
    linkNode *last;

    void connectBehind(linkNode *behind)
    {
        this->next = behind;
        behind->last = this;
    }

    void connectFront(linkNode *front)
    {
        this->last = front;
        front->next = this;
    }

    void deleteThis(void)
    {
        if(this->next != nullptr) this->next->last = this->last;
        if(this->last != nullptr) this->last->next = this->next;
        delete this;
    }

    ~linkNode()
    {
        // if((this->next != nullptr)&&(this->last != nullptr)) 
        // {
        //     this->next->last = this->last; 
        //     this->last->next = this->next;
        // }
    }
};



/**
 * 链表 
*/
template<typename theType>
class linkList
{
public:
    linkList()
    {
        this->head = nullptr;
        this->rear = nullptr;
    }

    linkList(linkNode<theType> *first)
    {
        this->head = first;
        this->rear = first;
    }

    linkList(linkNode<theType> *head, linkNode<theType> *rear)
    {
        this->head = head;
        this->rear = rear;
    }

    void push(linkNode<theType> *newNode)
    {
        if(this->rear == nullptr)
        {
            this->rear = newNode;
            this->head = newNode;
        }
        else
        {
            this->rear->connectBehind(newNode);
            this->rear = newNode;
        }
    }

    void pop(void)
    {
        if(this->rear != nullptr)
        {
            this->rear = this->rear->last;
            this->rear->next->deleteThis();
        }
    }

    void deleteNodes(void)
    {
        while((this->head != nullptr)&&(this->rear != nullptr)&&(this->head != this->rear))
        {
            this->pop();
        }

        if(this->head != nullptr) 
        {
            delete this->head;
        }
        if((this->rear != nullptr)&&(this->rear != this->head))     //防止重复释放
        {
            delete this->rear;
            this->rear = nullptr;
        }
        this->head = nullptr;
    }

    ~linkList()
    {
        // while((this->head != nullptr)&&(this->rear != nullptr)&&(this->head != this->rear))
        // {
        //     this->pop();
        // }

        // this->head->deleteThis();
        // this->rear->deleteThis();
    }

    linkList<theType> *unitLinkLists(linkList<theType> *another)
    {
        if(this->head == nullptr) 
        {
            return (new linkList(another->head, another->rear));
        }
        if(another->head == nullptr)
        {
            return (new linkList(this->head, this->rear));
        }

        this->rear->next = another->head;
        another->head->last = this->rear;

        return (new linkList(this->head, another->rear));
    }

    linkNode<theType> *head;
    linkNode<theType> *rear;
};


typedef struct point
{
    int x;
    int y;
} point;

typedef struct dimension3
{
    double x;
    double y;
    double z;
} dimension3;


/**
 * 边缘点集合
*/
class edgePointsArry
{
public:
    edgePointsArry()
    {
        point nullpoint;
        nullpoint.x = 0;
        nullpoint.y = 0;

        for(int i = 0; i < 50000; i++)
        {
            data[i] = nullpoint;
        }
    }

    point data[50000];
};

#endif
