#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

struct mesure {
   double data;
   int key;
   struct mesure *next;
};

struct sensor {
   struct mesure *firstmesure;
   int key;
   struct sensor *next;
};

//display the list
void printListMesure(struct mesure *head);
//insert link at the first location
void insertFirstMesure(int key, double data, struct mesure**head);
//delete first item
struct mesure* deleteFirstMesure(struct mesure**head);
void deleteAllMesures(struct mesure**head);
//is list empty
bool isEmptyMesure(struct mesure*head);
int lengthMesure(struct mesure*head);
//find a link with given key
struct mesure* findMesure(int key, struct mesure*head);
struct mesure* changeMesure(int key, double newdata, struct mesure*head);
//delete a link with given key
struct mesure* deleteoneMesure(int key, struct mesure *head);
void sortMesure(struct mesure *head);

void printListSensor(struct sensor *head);
void insertFirstSensor(int key, struct sensor**head);
bool isEmptySensor(struct sensor*head);
struct sensor* findSensor(int key, struct sensor*head);

void recordMesure(char* mesure, struct mesure**head);
void recordAllMesure(char* message, struct sensor**head);

//%%%%% Mesure %%%%%
void printListMesure(struct mesure *head) {
   struct mesure *ptr = (struct mesure*) malloc(sizeof(struct mesure));
	ptr = head;
   printf("[");
	
   //start from the beginning
   while(ptr != NULL) {
      printf("(%d, %f), ", ptr->key, ptr->data);
      ptr = ptr->next;
   }
	
   printf(" ]\n");
   free(ptr);
}

void insertFirstMesure(int key, double data, struct mesure**head) {
   //create a link
   struct mesure *link = (struct mesure*) malloc(sizeof(struct mesure));
	
   link->key = key;
   link->data = data;
	
   //point it to old first mesure
   link->next = *head;
	
   //point first to new first mesure
   *head = link;
}

struct mesure* deleteFirstMesure(struct mesure**head) {

   //save reference to first link
   struct mesure *tempLink = *head;
	struct mesure *link = *head ;
   //mark next to first link as first 
   *head = link->next;
	
   //return the deleted link
   return tempLink;
}

void deleteAllMesures(struct mesure**head) {
   while(!isEmptyMesure(*head)) {            
      struct mesure *temp = deleteFirstMesure(head);
      printf("\nDeleted value:");
      printf("(%d,%f) ",temp->key,temp->data);
      free(temp);
   }  
}

bool isEmptyMesure(struct mesure*head) {
   return head == NULL;
}

int lengthMesure(struct mesure*head) {
   int length = 0;
   struct mesure *current;
	
   for(current = head; current != NULL; current = current->next) {
      length++;
   }
	
   return length;
}

struct mesure* findMesure(int key, struct mesure*head) {

   //start from the first link
   struct mesure* current = head;

   //if list is empty
   if(head == NULL) {
      return NULL;
   }

   //navigate through list
   while(current->key != key) {
	
      //if it is last mesure
      if(current->next == NULL) {
         return NULL;
      } else {
         //go to next link
         current = current->next;
      }
   }      
	
   //if data found, return the current Link
   return current;
}

struct mesure* changeMesure(int key, double newdata, struct mesure*head){
   //start from the first link
   struct mesure* current = head;

   //if list is empty
   if(head == NULL) {
      return NULL;
   }

   //navigate through list
   while(current->key != key) {
	
      //if it is last mesure
      if(current->next == NULL) {
         return NULL;
      } else {
         //go to next link
         current = current->next;
      }
   }      
	
   //if data found, return the current Link
   current->data = newdata;
   return current;
}

//delete a link with given key
struct mesure* deleteoneMesure(int key, struct mesure *head) {

   //start from the first link
   struct mesure* current = head;
   struct mesure* previous = NULL;
	
   //if list is empty
   if(head == NULL) {
      return NULL;
   }

   //navigate through list
   while(current->key != key) {

      //if it is last mesure
      if(current->next == NULL) {
         return NULL;
      } else {
         //store reference to current link
         previous = current;
         //move to next link
         current = current->next;
      }
   }

   //found a match, update the link
   if(current == head) {
      //change first to point to next link
      head = head->next;
   } else {
      //bypass the current link
      previous->next = current->next;
   }    
	
   return current;
}

void sortMesure(struct mesure *head) {

   int i, j, k, tempKey, tempData;
   struct mesure *current;
   struct mesure *next;
	
   int size = lengthMesure(head);
   k = size ;
	
   for ( i = 0 ; i < size - 1 ; i++, k-- ) {
      current = head;
      next = head->next;
		
      for ( j = 1 ; j < k ; j++ ) {   

         if ( current->data > next->data ) {
            tempData = current->data;
            current->data = next->data;
            next->data = tempData;

            tempKey = current->key;
            current->key = next->key;
            next->key = tempKey;
         }
			
         current = current->next;
         next = next->next;
      }
   }   
}

void reverseMesure(struct mesure** head_ref) {
   struct mesure* prev   = NULL;
   struct mesure* current = *head_ref;
   struct mesure* next;
	
   while (current != NULL) {
      next  = current->next;
      current->next = prev;   
      prev = current;
      current = next;
   }
	
   *head_ref = prev;
}

void recordMesure(char* mesure, struct mesure**head){
   char separateur = ';';
   char buffer[strlen(mesure)];
   strcpy(buffer,mesure);
   int nb = 1;
   int i = 0;
   while (mesure[i] != '\0') {
      if (mesure[i] == separateur){
         mesure = &mesure[i+1];
         buffer[i]='\0';
         insertFirstMesure(nb,atof(buffer),head);
         strcpy(buffer, mesure);
         nb++;
         i=0;
      }
      else{
         i++;
      }
   }
   insertFirstMesure(nb,atof(buffer),head);
}

void recordAllMesure(char* message, struct sensor**head){
   char separateur = ';';
   char buffer[strlen(message)];
   int nb = 1;
   int i = 0;
   strcpy(buffer,message);
   
   while (message[i] != '\0') {
      if (message[i] == '['){
         message = &message[i+1];
         strcpy(buffer, message);
         i=0;
      }
      else{
         if(message[i] == ']'){
            message = &message[i+1];
            buffer[i]='\0';
            insertFirstSensor(nb,head);
            recordMesure(buffer, &((*head)->firstmesure));
            strcpy(buffer, message);
            nb++;
            i=0;
         }else{
            i++;
         }
      }
   }
}


//%%%%% Sensor %%%%%
void printListSensor(struct sensor *head) {
   struct sensor *ptr = head;
   printf("[ ");
	
   //start from the beginning
   while(ptr != NULL) {
      printListMesure(ptr->firstmesure);
      ptr = ptr->next;
   }
	
   printf(" ]\n");
}

void insertFirstSensor(int key, struct sensor**head) {
   //create a link
   struct sensor *link = (struct sensor*) malloc(sizeof(struct sensor));
	
   link->key = key;
   link->firstmesure = NULL;
	
   //point it to old first sensor
   link->next = *head;
	
   //point first to new first sensor
   *head = link;
}

bool isEmptySensor(struct sensor*head) {
   return head == NULL;
}

struct sensor* findSensor(int key, struct sensor*head) {

   //start from the first link
   struct sensor* current = head;

   //if list is empty
   if(head == NULL) {
      return NULL;
   }

   //navigate through list
   while(current->key != key) {
	
      //if it is last node
      if(current->next == NULL) {
         return NULL;
      } else {
         //go to next link
         current = current->next;
      }
   }      
	
   //if data found, return the current Link
   return current;
}

