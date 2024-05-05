int prev_OS = -1, prev_IS = -1, prev_treshold = -1;
bool fuzzy_treshold(int AMOUNT_OF_TARGETS, short OS, short IS)
{
   //printf("value of IS is %d\n" ,IS);
   bool onlooker = false;
   int treshold = 0;
   int IS_TRESHOLD = 3;
   int OS_TRESHOLD = 3;
   
   int IS_value, OS_value;

   if (IS > IS_TRESHOLD) // Triggered when IS = 2,3,4,5,6 
   {
      IS_value = -1;
   }
   else if (IS <= IS_TRESHOLD) // Triggered when IS = 0,1
   {
      IS_value = 1;
   }
      if (OS < OS_TRESHOLD)
   {
      OS_value = -1;
   }
   else if (OS >= OS_TRESHOLD)
   {
      OS_value = 1;
   }

   //treshold = BS_value  + SS_value + IS_value;
     treshold = IS_value;
   if (prev_OS != OS_value || prev_IS != IS_value || prev_treshold != treshold)
   {
      prev_IS = IS_value;
      prev_OS = OS_value;
      prev_treshold = treshold;
   }

   ////////////////////////
   if (treshold < 0)
   {
      onlooker = false;
   }
   else if (treshold >= 0)
   {
      onlooker = true;
   }

   return onlooker;
}
