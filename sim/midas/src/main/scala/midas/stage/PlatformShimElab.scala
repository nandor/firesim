// See LICENSE for license details.

package midas.stage

import java.io.{File, FileWriter}

import firrtl._
import firrtl.ir._
import firrtl.annotations.{Annotation, CircuitName, ReferenceTarget, ModuleTarget, InstanceTarget, JsonProtocol, TargetToken}
import firrtl.Mappers._

import freechips.rocketchip.config.{Parameters, Config}
import freechips.rocketchip.diplomacy.{LazyModule}

import midas.core._
import midas.passes.ElaborateChiselSubCircuit
import midas.platform.PlatformShim
import midas.Platform

import org.json4s._
import org.json4s.native.JsonMethods._
import org.json4s.jackson.Serialization.write

case class ElabParams(
  annotations: Seq[Annotation],
  portTypes: Seq[(String, String)],
  moduleName: String
)

object ElabParams {
  def deserialize(in: JsonInput): ElabParams = {
    case class Raw(annotations: JArray, portTypes: Seq[(String, String)], moduleName: String)
    implicit val formats: Formats = DefaultFormats
    val parsed = parse(in)
    val raw = parsed.extract[Raw]
    new ElabParams(JsonProtocol.deserialize(write(raw.annotations)), raw.portTypes, raw.moduleName)
  }
}

object PlatformShimElabMain {
  def main(args: Array[String]) = {
    // TODO: Somehow this needs to be fed in from outside.
    val parameters = new midas.F1Config

    // TODO: We likely need to do what `generateHeaderAnnos` does in
    // `SimulationMapping.scala`.

    // Deserialize the elaboration parameters from the JSON input.
    var elabParamsFile: Option[String] = None
    var outputFirFile: Option[String] = None
    var outputAnnoFile: Option[String] = None
    def parseOption(list: List[String]): Unit = list match {
      case "-i" :: fname :: tail =>
        elabParamsFile = Some(fname)
        parseOption(tail)
      case "-o" :: firName :: annoName :: tail =>
        outputFirFile = Some(firName)
        outputAnnoFile = Some(annoName)
        parseOption(tail)
      case option :: tail => throw new Exception("unknown argument `" + option + "`")
      case Nil => Unit
    }
    parseOption(args.toList)
    val elabParams = elabParamsFile match {
      case None => ElabParams.deserialize(System.in)
      case Some(fname) => {
        val file = scala.io.Source.fromFile(fname)
        val p = ElabParams.deserialize(file.mkString)
        file.close()
        p
      }
    }

    // The platform shim requires a mapping from the top-level channel ports to
    // the actual types of those ports. To make the `PlatformShim` interface
    // happy without modifying its code, we map the top-level port names to the
    // `ReferenceTarget`s that make up the map keys, and parse the type string
    // into the actual FIRRTL type and then wrap it in a mock FIRRTL `Port`.
    //
    // TODO: Make `PlatformShim` operate on types instead of ports.
    val portTypeMap: Map[ReferenceTarget, Port] = elabParams.portTypes.map { case (ref, ty) =>
      val target = ReferenceTarget(elabParams.moduleName, elabParams.moduleName, Nil, ref, Nil)
      val port = Port(NoInfo, "dummy", Input, firrtl.Parser.parseType(ty))
      (target, port)
    }.toMap

    // Generate and elaborate the platform shim.
    lazy val shim = PlatformShim(elabParams.annotations, portTypeMap)(parameters)
    val (chirrtl, elaboratedAnnos) = ElaborateChiselSubCircuit(LazyModule(shim).module)

    // Map the `portTypes` field of the elaboration parameters to a list of port
    // names in their desired order, which we'll use later to reshuffle the
    // generated extmodule such that it matches what the elaboration parameters
    // request.
    val portOrder = elabParams.portTypes.map(x => x._1)

    // The shim instantiates a `TargetBox` extmodule that we have to link
    // against the top-level of the actual design we're generating a wrapper
    // for. Use the `TargetBoxAnnotation` to figure out where that extmodule is
    // and rename it to match the thing we're linking against.
    val boxTarget = elaboratedAnnos.collectFirst({
      case TargetBoxAnnotation(it: InstanceTarget) => it
    }).getOrElse(throw new Exception("TargetBoxAnnotation not found or annotated top module!"))
    val updatedChirrtl = chirrtl.mapModule(replaceTargetBox(
      elabParams.moduleName,
      boxTarget.encapsulatingModule,
      boxTarget.ofModule,
      boxTarget.instance,
      portOrder
    ))
    val updatedAnnos = elaboratedAnnos.filter({
      case TargetBoxAnnotation(_) => false
      case _ => true
    })

    // Dump the generated CHIRRTL.
    val circuitFir = updatedChirrtl.serialize
    outputFirFile match {
      case Some(fname) =>
        val fw = new FileWriter(new File(fname))
        fw.write(circuitFir)
        fw.close()
      case None =>
        println(circuitFir)
    }

    // Dump the annotations.
    val annosJson = JsonProtocol.serialize(updatedAnnos)
    outputAnnoFile match {
      case Some(fname) =>
        val fw2 = new FileWriter(new File(fname))
        fw2.write(annosJson)
        fw2.close()
      case None =>
        println(annosJson)
    }
  }

  // Replace the target box instance and extmodule with the name of the actual
  // FPGA top-level module to be instantiated.
  private def replaceTargetBox(
    desiredName: String,
    boxParent: String,
    boxModule: String,
    boxInst: String,
    portOrder: Seq[String]
  )(m: DefModule) = m match {
    case m: Module if m.name == boxParent =>
      m.mapStmt(replaceTargetBoxStmt(desiredName, boxInst))
    case m: ExtModule if m.name == boxModule => {
      val portPositions = portOrder.zipWithIndex.toMap
      def portPosition(port: Port) = portPositions.get(port.name).getOrElse(
        throw new Exception("Unexpected port " + port)
      )
      val ports = m.ports.sortWith(portPosition(_) < portPosition(_))
      m.copy(name = desiredName, defname = desiredName, ports = ports)
    }
    case m => m
  }

  // Replace the target box instance to target the actual FPGA top-level module.
  private def replaceTargetBoxStmt(
    desiredName: String,
    boxInst: String
  )(s: Statement): Statement = s match {
    case s @ WDefInstance(_, name, _, _) if name == boxInst =>
      s copy (module = desiredName)
    case s => s map replaceTargetBoxStmt(desiredName, boxInst)
  }
}
