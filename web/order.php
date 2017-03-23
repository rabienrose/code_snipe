<?php
ini_set("display_errors", "On");
error_reporting(E_ALL | E_STRICT);
header("Content-Type:text/html;charset=utf-8");
class Order{
	function checkNoon($curTime){
		$timeNow= ($curTime/3600+0)/24;
		$dayPartition = $timeNow - floor($timeNow);
		$dayPartition = $dayPartition*24;
		if ($dayPartition <11){
			return false;
		}else{
			return true;
		}
	}
	
	function checkDupDish($dishIds, $tarIds){
		$dishIdsCount = count($dishIds);
		$dup = false;
		for ($i=0; $i< $dishIdsCount; $i++){
			if($dishIds[$i] == $tarIds){
				$dup = true;
				break;
			}
		}
		return $dup;
	}
	
	function getDishIdsByTable($tableId, $con, $timeNow){
		$sql = "select orderT, dishId from user where tableId=".$tableId;
		$tableDishRe = $con->query($sql);
		$tablDishList = array();
		$i=0;
		while($dishIdInAll = $tableDishRe->fetch_row()){
			$orderT = $dishIdInAll[0];
			$dishId = $dishIdInAll[1];
			if ($dishId<0){
				continue;
			}
			$diffTime = $timeNow - $orderT;
			$diffTime=$diffTime/3600;
			if ($diffTime >12){
				continue;
			}
			$tablDishList[$i] = $dishId;
			$i++;
		}
		return $tablDishList;
	}
	
	function getAvilDishes($DisAvilDishIds, $con){
		$sql = "select id from menu";
		$allDishIds = $con->query($sql);
		$avilDishList = array();
		$i=0;
		while($dishIdInAll = $allDishIds->fetch_row()){
			if ($this->checkDupDish($DisAvilDishIds, $dishIdInAll[0])==false){
				$avilDishList[$i] = $dishIdInAll[0];
				$i++;
			}
		}
		return $avilDishList;
	}
	
	public function addUser($id, $name){
		$dbHelper = new DBHelper();
		$con = $dbHelper->openDataBase();
		$sql = "select id from user where id='".$id."'";
		$result = $con->query($sql);
		var_dump($name);
		if ($result->num_rows>0){
			$sql="update user set name='".$name."' where id='".$id."'";
			$con->query($sql);
		}else{
			
			$sql="INSERT INTO user (id, name)
						VALUES ('".$id."','".$name."')";
			$con->query($sql);
		}
		$dbHelper->closeDataBase($con);
	}
	
	public function showUser(){
		$reStr="";
		$dbHelper = new DBHelper();
		$con = $dbHelper->openDataBase();
		$sql="select id, name from user";
		$result = $con->query($sql);
		while($row = $result->fetch_row()){
			
			$reStr .= $row[0]." ".$row[1];
			$reStr .= '</br>';
		}
		$dbHelper->closeDataBase($con);
		return $reStr;
	}
	
	public function setTable($userId, $tableId){
		$dbHelper = new DBHelper();
		$con = $dbHelper->openDataBase();
		$sql = "select id from user where id='".$userId."'";
		$result = $con->query($sql);
		if ($result->num_rows>0){
			$sql="update user set tableId='".$tableId."' where id='".$userId."'";
			$con->query($sql);
		}
		$dbHelper->closeDataBase($con);
	}
	
	public function showDishes($tableId){
		$reStr = "";
		$timeNow = time();
		$dbHelper = new DBHelper();
		$con = $dbHelper->openDataBase();
		$sql = "select name, orderT, dishId from user where tableId=".$tableId;
		$result = $con->query($sql);
		if(!$result){
			$reStr .= "没有此桌！！";
			return;
		}
		while($row = $result->fetch_row()){
			$name = $row[0];
			$orderT = $row[1];
			$dishId = $row[2];
			if ($dishId<0){
				continue;
			}
			$diffTime = $timeNow - $orderT;
			$diffTime=$diffTime/3600;
			if ($diffTime >12){
				continue;
			}
			$sql="select name from menu where id=".$dishId;
			$menuRe = $con->query($sql);
			if ($menuRe->num_rows>0){
				$menuItem = $menuRe->fetch_row();
				echo $name."点了".$menuItem[0]."</br>";
			}
		}
		$isNoon = $this->checkNoon($timeNow);
		if ($isNoon){
			$reStr .= "今日菜谱已定！";
		}else{
			$reStr .= "菜谱持续跟新中。。。";
		}
		$dbHelper->closeDataBase($con);
		return $reStr;
	}
	
	public function imIn($userId, $dishId){
		$reStr = "";
		$timeNow = time();
		$isNoon = $this->checkNoon($timeNow);
		if ($isNoon){
			$reStr .= "���ձ����ѽ���"."</br>";
			return;	
		}
		$dbHelper = new DBHelper();
		$con = $dbHelper->openDataBase();
		
		$sql = "select id, tableId from user where id='".$userId."'";
		$result = $con->query($sql);
		if (!$result){
			return;
		}	
		
		$userItem = $result->fetch_row();
		$tableId = $userItem[1];
		$tableDish = $this->getDishIdsByTable($tableId, $con, $timeNow);
		if ($dishId<0){
			$avilDish = $this->getAvilDishes($tableDish, $con);
			$avilCount = count($avilDish);
			if ($avilCount ==0){
				echo "û�пɵ�Ĳ��ˣ���";
				return;
			}
			$randRe = mt_rand(0, $avilCount-1);
			$dishId = $avilDish[$randRe];
		}else{
			if ($this->checkDupDish($tableDish, $dishId)){
				$reStr .= "�˲��ѵ㣡��";
				return;
			}
		}
		$sql="update user set dishId='".$dishId."' where id='".$userId."'";
		$con->query($sql);
		$sql="update user set orderT='".$timeNow."' where id='".$userId."'";
		$con->query($sql);
		$reStr .= "����ɹ���";
		$reStr .='</br>';
		$dbHelper->closeDataBase($con);
		return $reStr;
	}
	
	public function imOut($userId){
		$dbHelper = new DBHelper();
		$con = $dbHelper->openDataBase();
		$sql = "select id from user where id='".$userId."'";
		$result = $con->query($sql);
		if ($result->num_rows>0){
			$sql="update user set dishId=-1 where id='".$userId."'";
			$con->query($sql);
		}
		$dbHelper->closeDataBase($con);
	}
	
	public function addMenu($id, $dish, $price, $type){
		$dbHelper = new DBHelper();
		$con = $dbHelper->openDataBase();
		$sql="select id from menu where id='".$id."'";
		$result = $con->query($sql);
		if ($result->num_rows>0){
			$sql="update menu set name='".$dish."', price=".$price.", type=".$type." where id=".$id;
			$con->query($sql);
		}else{
			$sql="INSERT INTO menu (id, name, price, type)
						VALUES ('".$id."','".$dish."','".$price."','".$type."')";
			$con->query($sql);
		}
		$dbHelper->closeDataBase($con);
	}
	
	public function delMenu($id){
		$dbHelper = new DBHelper();
		$con = $dbHelper->openDataBase();
		$sql="select id from menu where id='".$id."'";
		$result = $con->query($sql);
		if ($result->num_rows>0){
			$sql="delete from menu where id=".$id;
			$con->query($sql);
		}
		$dbHelper->closeDataBase($con);
	}
	
	public function showMenu(){
		$dbHelper = new DBHelper();
		$con = $dbHelper->openDataBase();
		$sql="select * from menu";
		$result = $con->query($sql);
		$row = $result->fetch_row();
		while($row != NULL){
			echo $row[0]." ".$row[1]." ".$row[2]." ".$row[3];
			echo '</br>';
			$row = $result->fetch_row();
		}
		$dbHelper->closeDataBase($con);
	}
}

class DBHelper{
	public function openDataBase(){
		$con = new mysqli('bdm246008380.my3w.com', 'bdm246008380', 'la009296','bdm246008380_db');
		return $con;
	}
	
	public function closeDataBase($con){
		$con->close();
	}
}
?>