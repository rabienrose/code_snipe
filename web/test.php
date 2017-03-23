<?php
include 'order.php';
$myOrder = new Order();
$inputStr="";
$myOrder->addUser("ssss", "王梓");
$myOrder->addUser("aaaa", "代承丽");
$outStr = $myOrder->showUser();
echo $outStr;
$testStr = "王梓里";
var_dump($testStr);
echo "$testStr[0]$testStr[1]$testStr[2]";
?>